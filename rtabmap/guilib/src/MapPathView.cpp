/*
 * MapPathView.cpp - see MapPathView.h.
 *
 * Rendering mirrors the pygame viewer in path_planning/prototype_prd3.py
 * (Viewer._draw_map / _draw_overlays / _draw_panel): the same colour legend,
 * the yellow planned path, the green goal ring and the blue robot pose + heading.
 */
#include "MapPathView.h"

#include <QColor>
#include <QFont>
#include <QFontMetrics>
#include <QImage>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QPainter>
#include <QPen>
#include <QPolygonF>
#include <QString>

#include <algorithm>
#include <chrono>
#include <cmath>

namespace rtabmap {

MapPathView::MapPathView(QWidget * parent) :
	QWidget(parent)
{
	setMinimumSize(320, 240);
	setFocusPolicy(Qt::StrongFocus);
	setAttribute(Qt::WA_OpaquePaintEvent, true);
	repaintThrottle_.start();
	worker_ = std::thread(&MapPathView::workerLoop, this);
}

MapPathView::~MapPathView()
{
	stop_ = true;
	inCv_.notify_all();
	if(worker_.joinable())
		worker_.join();
}

void MapPathView::updateGrid(const cv::Mat & map8S, float xMin, float yMin, float cellSize,
                             float poseX, float poseY, float poseYaw)
{
	if(map8S.empty() || map8S.type() != CV_8SC1)
		return;

	cv::Mat g = map8S.clone(); // own a copy; the caller reuses its buffer next frame
	GridMeta m;
	m.w = map8S.cols; m.h = map8S.rows;
	m.xMin = xMin; m.yMin = yMin; m.cell = cellSize;
	m.px = poseX; m.py = poseY; m.yaw = poseYaw;

	// Realtime display (GUI thread): refresh the painted map immediately, decoupled
	// from the D* worker. cv::Mat is refcounted, so this shares g's buffer (no copy).
	dispGrid_ = g; dispMeta_ = m; haveDisp_ = true; ++dispSeq_;

	bool firstGrid;
	{
		std::lock_guard<std::mutex> lk(inMutex_);
		firstGrid = !haveLast_;
		lastGrid_ = g; lastMeta_ = m; haveLast_ = true; // keep only the latest grid
	}
	// Don't replan on every grid frame — the worker plans on its own 2 s cadence.
	// Only kick it for the very first grid so the initial path appears immediately.
	if(firstGrid)
	{
		forceReplan_ = true;
		inCv_.notify_one();
	}
	// Repaint now (throttled ~30 Hz, shared with updatePose) so map + pose update in
	// real time; the per-cell image rebuild only runs when dispSeq_ advances.
	if(!repaintThrottle_.isValid() || repaintThrottle_.elapsed() >= 33)
	{
		repaintThrottle_.restart();
		update();
	}
}

void MapPathView::workerLoop()
{
	PathGenerator gen;
	while(!stop_)
	{
		cv::Mat grid;
		GridMeta meta;
		bool hasGoal = false;
		float gxw = 0.f, gyw = 0.f;
		{
			std::unique_lock<std::mutex> lk(inMutex_);
			// Fixed 2 s replan cadence: wake on the timeout, or early for an
			// explicit replan request (first grid / manual goal / 'R' reset).
			inCv_.wait_for(lk, std::chrono::milliseconds(2000),
			               [this]{ return stop_ || forceReplan_.load(); });
			if(stop_)
				break;
			forceReplan_ = false;
			if(!haveLast_)
				continue;            // no grid yet — keep waiting
			grid = lastGrid_;        // always plan on the latest available grid
			meta = lastMeta_;
			hasGoal = hasManualGoal_;
			gxw = manualGoalX_;
			gyw = manualGoalY_;
		}

		if(resetRequested_.exchange(false))
			gen.reset();

		std::vector<std::pair<float, float> > worldPath =
			gen.compute(meta, grid, hasGoal, gxw, gyw);

		std::shared_ptr<Snapshot> snap = std::make_shared<Snapshot>();
		snap->grid = grid;
		snap->meta = meta;
		snap->pathCells = gen.pathCells();
		snap->worldPath = worldPath;
		snap->hasStart = gen.hasStart();
		snap->startCell = gen.startCell();
		snap->hasGoal = gen.hasGoal();
		snap->goalCell = gen.goalCell();
		snap->goalSrc = (int)gen.goalSrc();
		// Goal in world metres (north-up cell -> world), so paintEvent can place it on
		// the live display grid even after the grid origin scrolls past plan time.
		if(snap->hasGoal)
		{
			snap->goalX = meta.xMin + (snap->goalCell.first + 0.5f) * meta.cell;
			snap->goalY = meta.yMin + ((meta.h - 1 - snap->goalCell.second) + 0.5f) * meta.cell;
		}
		for(int y=0; y<grid.rows; ++y)
		{
			const signed char * r = grid.ptr<signed char>(y);
			for(int x=0; x<grid.cols; ++x)
			{
				signed char v = r[x];
				if(v == kCellUnknown)           ++snap->nUnknown;
				else if(v == kCellFree)         ++snap->nFree;
				else if(v == kCellSemanticWall) ++snap->nSemantic;
				else if(v == kCellExit)         ++snap->nExit;
				else if(v == kCellObstacle)     ++snap->nObstacle;
			}
		}

		{
			std::lock_guard<std::mutex> lk(outMutex_);
			snapshot_ = snap;
		}
		QMetaObject::invokeMethod(this, [this]{ update(); }, Qt::QueuedConnection);
	}
}

double MapPathView::scaleFor(int w, int h, double & ox, double & oy) const
{
	if(w <= 0 || h <= 0)
	{
		ox = oy = 0.0;
		return 1.0;
	}
	double sx = (double)width()  / (double)w;
	double sy = (double)height() / (double)h;
	double scale = std::min(sx, sy);
	if(scale <= 0.0)
		scale = 1.0;
	ox = (width()  - w * scale) * 0.5;
	oy = (height() - h * scale) * 0.5;
	return scale;
}

void MapPathView::paintEvent(QPaintEvent *)
{
	QPainter p(this);
	p.fillRect(rect(), QColor(30, 30, 30));

	// Realtime display grid (GUI thread): the map + pose are drawn from here every
	// frame, independent of the async D* snapshot below.
	if(!haveDisp_ || dispGrid_.empty())
	{
		p.setPen(QColor(200, 200, 200));
		p.drawText(rect(), Qt::AlignCenter, tr("Waiting for map..."));
		return;
	}
	const GridMeta meta = dispMeta_;
	const int w = meta.w, h = meta.h;

	// Async plan snapshot (path / goal / stats); may be null or older than the grid.
	std::shared_ptr<Snapshot> snap;
	{
		std::lock_guard<std::mutex> lk(outMutex_);
		snap = snapshot_;
	}

	// Live pose (map frame), fed at odometry rate — real-time, independent of the
	// D* snapshot. Falls back to the grid frame's plan-time pose until one arrives.
	float lpx, lpy, lpyaw; bool lpHave;
	{
		std::lock_guard<std::mutex> lk(poseMutex_);
		lpx = liveX_; lpy = liveY_; lpyaw = liveYaw_; lpHave = havePose_;
	}
	if(!lpHave)
	{
		lpx = meta.px; lpy = meta.py; lpyaw = meta.yaw;
	}

	// --- map cells (flipped so north is up, matching the planner & pygame view) ---
	// Rebuilt only when the display grid changes; pose-rate repaints reuse the cache
	// so the per-cell loop doesn't run on every odometry tick.
	if(dispSeq_ != lastDispSeq_ || mapImage_.width() != w || mapImage_.height() != h)
	{
		QImage img(w, h, QImage::Format_RGB888);
		for(int y=0; y<h; ++y)
		{
			const signed char * src = dispGrid_.ptr<signed char>(h - 1 - y);
			uchar * dst = img.scanLine(y);
			for(int x=0; x<w; ++x)
			{
				int r, g, b;
				switch(src[x])
				{
					case kCellFree:         r=210; g=210; b=210; break; // free
					case kCellSemanticWall: r=200; g= 60; b= 60; break; // semantic wall
					case kCellExit:         r= 50; g=220; b= 70; break; // exit
					case kCellObstacle:     r= 20; g= 20; b= 20; break; // obstacle
					default:                r= 60; g= 60; b= 60; break; // unknown / other
				}
				dst[x*3 + 0] = (uchar)r;
				dst[x*3 + 1] = (uchar)g;
				dst[x*3 + 2] = (uchar)b;
			}
		}
		mapImage_ = img;
		lastDispSeq_ = dispSeq_;
	}

	double ox, oy;
	double scale = scaleFor(w, h, ox, oy);
	p.setRenderHint(QPainter::SmoothPixmapTransform, false);
	p.drawImage(QRectF(ox, oy, w * scale, h * scale), mapImage_);

	// world (metres) -> screen against the live display grid. Inverse of the planner's
	// cell<->world mapping in this flipped/north-up frame. Path/goal/pose are all drawn
	// in world coords so they stay correctly placed as the grid refreshes.
	auto worldToScreen = [&](float wx, float wy) {
		double colf = (wx - meta.xMin) / meta.cell;
		double rowf = h - (wy - meta.yMin) / meta.cell;
		return QPointF(ox + colf * scale, oy + rowf * scale);
	};

	// --- planned path (yellow) + end marker --- world coords, from the async snapshot.
	if(snap && snap->worldPath.size() > 1)
	{
		QPen pen(QColor(255, 200, 0));
		pen.setWidthF(std::max(2.0, scale / 10.0));
		pen.setJoinStyle(Qt::RoundJoin);
		p.setPen(pen);
		QPolygonF poly;
		for(size_t i=0; i<snap->worldPath.size(); ++i)
			poly << worldToScreen(snap->worldPath[i].first, snap->worldPath[i].second);
		p.drawPolyline(poly);
		p.setBrush(QColor(255, 200, 0));
		p.drawEllipse(poly.back(), std::max(4.0, scale / 4.0), std::max(4.0, scale / 4.0));
		p.setBrush(Qt::NoBrush);
	}

	// --- goal ring (green) --- world coords, from the async snapshot.
	if(snap && snap->hasGoal)
	{
		QPen pen(QColor(50, 230, 50));
		pen.setWidthF(2.0);
		p.setPen(pen);
		double rr = std::max(5.0, scale / 3.0);
		p.drawEllipse(worldToScreen(snap->goalX, snap->goalY), rr, rr);
	}

	// --- robot pose (blue) dot + heading line --- (live pose, real-time) ---
	{
		QPointF a = worldToScreen(lpx, lpy);
		double rr = std::max(4.0, scale / 3.0);
		p.setPen(Qt::NoPen);
		p.setBrush(QColor(60, 120, 255));
		p.drawEllipse(a, rr, rr);
		double ln = std::max(scale, 12.0);
		// screen y grows downward -> negate sin (same as the pygame viewer)
		QPointF tip(a.x() + ln * std::cos(lpyaw), a.y() - ln * std::sin(lpyaw));
		QPen pen(QColor(60, 120, 255));
		pen.setWidthF(std::max(2.0, scale / 12.0));
		p.setPen(pen);
		p.drawLine(a, tip);
		p.setBrush(Qt::NoBrush);
	}

	// --- info overlay (top-left translucent box) ---
	const int goalSrc = snap ? snap->goalSrc : 0;
	const char * goalLabel = goalSrc == PathGenerator::GOAL_EXIT   ? "exit tag (10)" :
	                         goalSrc == PathGenerator::GOAL_MANUAL ? "manual (click)" : "none";
	QStringList lines;
	lines << QString("grid %1x%2  cell %3 m").arg(w).arg(h).arg(meta.cell, 0, 'f', 3);
	lines << QString("pose (%1, %2)  yaw %3")
	             .arg(lpx, 0, 'f', 2).arg(lpy, 0, 'f', 2)
	             .arg(lpyaw * (180.0 / 3.14159265358979323846), 0, 'f', 1);
	lines << QString("goal: %1   path: %2 pts")
	             .arg(goalLabel).arg(snap ? (int)snap->worldPath.size() : 0);
	lines << QString("obstacle %1  semantic %2  exit %3")
	             .arg(snap ? snap->nObstacle : 0).arg(snap ? snap->nSemantic : 0)
	             .arg(snap ? snap->nExit : 0);
	lines << QString("[L-click] goal   [R] replan");

	QFont f = p.font();
	f.setPointSize(8);
	p.setFont(f);
	QFontMetrics fm(f);
	int lh = fm.height() + 2;
	int boxW = 0;
	for(const QString & s : lines)
		boxW = std::max(boxW, fm.horizontalAdvance(s));
	boxW += 12;
	int boxH = lh * lines.size() + 8;
	p.setPen(Qt::NoPen);
	p.setBrush(QColor(0, 0, 0, 150));
	p.drawRect(6, 6, boxW, boxH);
	p.setPen(QColor(235, 235, 235));
	for(int i=0; i<lines.size(); ++i)
		p.drawText(12, 6 + 4 + fm.ascent() + i * lh, lines[i]);
}

void MapPathView::mousePressEvent(QMouseEvent * event)
{
	setFocus();
	if(event->button() != Qt::LeftButton)
		return;

	if(!haveDisp_ || dispGrid_.empty())
		return;

	const GridMeta meta = dispMeta_;   // convert against the displayed (live) grid
	const int w = meta.w, h = meta.h;
	double ox, oy;
	double scale = scaleFor(w, h, ox, oy);
	int col = (int)((event->x() - ox) / scale);
	int row = (int)((event->y() - oy) / scale);     // flipped/north-up row
	if(col < 0 || col >= w || row < 0 || row >= h)
		return;

	// cell -> world (inverse of the planner's world_to_cell)
	float wx = meta.xMin + (col + 0.5f) * meta.cell;
	float wy = meta.yMin + ((h - 1 - row) + 0.5f) * meta.cell;

	bool ready;
	{
		std::lock_guard<std::mutex> lk(inMutex_);
		hasManualGoal_ = true;
		manualGoalX_ = wx;
		manualGoalY_ = wy;
		ready = haveLast_;
	}
	if(ready) // user action -> replan immediately, don't wait for the 2 s tick
	{
		forceReplan_ = true;
		inCv_.notify_one();
	}
}

void MapPathView::keyPressEvent(QKeyEvent * event)
{
	if(event->key() == Qt::Key_R)
	{
		resetRequested_ = true;
		forceReplan_ = true; // replan immediately on the latest grid
		inCv_.notify_one();
	}
	else
	{
		QWidget::keyPressEvent(event);
	}
}

void MapPathView::updatePose(float poseX, float poseY, float poseYaw)
{
	{
		std::lock_guard<std::mutex> lk(poseMutex_);
		liveX_ = poseX; liveY_ = poseY; liveYaw_ = poseYaw; havePose_ = true;
	}
	// Called on the GUI thread (from MainWindow::processOdometry) — repaint directly.
	// Throttle to ~30 Hz so a fast odometry stream doesn't flood the event loop; the
	// map image is cached, so this repaint only re-draws the marker/overlay.
	if(!repaintThrottle_.isValid() || repaintThrottle_.elapsed() >= 33)
	{
		repaintThrottle_.restart();
		update();
	}
}

bool MapPathView::latestPlanAndPose(std::vector<std::pair<float, float> > & worldPath,
                                    float & px, float & py, float & yaw) const
{
	{
		std::lock_guard<std::mutex> lk(outMutex_);
		if(!snapshot_)
			return false;
		worldPath = snapshot_->worldPath;
	}
	std::lock_guard<std::mutex> lk(poseMutex_);
	if(!havePose_ || worldPath.empty())
		return false;
	px = liveX_; py = liveY_; yaw = liveYaw_;
	return true;
}

} // namespace rtabmap
