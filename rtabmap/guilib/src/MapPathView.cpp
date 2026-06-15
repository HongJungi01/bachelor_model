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
#include <cmath>

namespace rtabmap {

MapPathView::MapPathView(QWidget * parent) :
	QWidget(parent)
{
	setMinimumSize(320, 240);
	setFocusPolicy(Qt::StrongFocus);
	setAttribute(Qt::WA_OpaquePaintEvent, true);
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

	{
		std::lock_guard<std::mutex> lk(inMutex_);
		lastGrid_ = g; lastMeta_ = m; haveLast_ = true;
		pendingGrid_ = g; pendingMeta_ = m; pendingDirty_ = true;
	}
	inCv_.notify_one();
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
			inCv_.wait(lk, [this]{ return pendingDirty_ || stop_; });
			if(stop_)
				break;
			grid = pendingGrid_;
			meta = pendingMeta_;
			pendingDirty_ = false;
			hasGoal = hasManualGoal_;
			gxw = manualGoalX_;
			gyw = manualGoalY_;
		}

		if(resetRequested_.exchange(false))
			gen.reset();

		gen.compute(meta, grid, hasGoal, gxw, gyw);

		std::shared_ptr<Snapshot> snap = std::make_shared<Snapshot>();
		snap->grid = grid;
		snap->meta = meta;
		snap->pathCells = gen.pathCells();
		snap->hasStart = gen.hasStart();
		snap->startCell = gen.startCell();
		snap->hasGoal = gen.hasGoal();
		snap->goalCell = gen.goalCell();
		snap->goalSrc = (int)gen.goalSrc();
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

	std::shared_ptr<Snapshot> snap;
	{
		std::lock_guard<std::mutex> lk(outMutex_);
		snap = snapshot_;
	}

	if(!snap || snap->grid.empty())
	{
		p.setPen(QColor(200, 200, 200));
		p.drawText(rect(), Qt::AlignCenter, tr("Waiting for map..."));
		return;
	}

	const int w = snap->meta.w, h = snap->meta.h;

	// --- map cells (flipped so north is up, matching the planner & pygame view) ---
	QImage img(w, h, QImage::Format_RGB888);
	for(int y=0; y<h; ++y)
	{
		const signed char * src = snap->grid.ptr<signed char>(h - 1 - y);
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

	double ox, oy;
	double scale = scaleFor(w, h, ox, oy);
	p.setRenderHint(QPainter::SmoothPixmapTransform, false);
	p.drawImage(QRectF(ox, oy, w * scale, h * scale), img);

	const std::pair<int,int> * sc = &snap->startCell;
	auto toScreen = [&](int col, int row) {
		return QPointF(ox + (col + 0.5) * scale, oy + (row + 0.5) * scale);
	};

	// --- planned path (yellow) + end marker ---
	if(snap->pathCells.size() > 1)
	{
		QPen pen(QColor(255, 200, 0));
		pen.setWidthF(std::max(2.0, scale / 10.0));
		pen.setJoinStyle(Qt::RoundJoin);
		p.setPen(pen);
		QPolygonF poly;
		for(size_t i=0; i<snap->pathCells.size(); ++i)
			poly << toScreen(snap->pathCells[i].first, snap->pathCells[i].second);
		p.drawPolyline(poly);
		p.setBrush(QColor(255, 200, 0));
		p.drawEllipse(poly.back(), std::max(4.0, scale / 4.0), std::max(4.0, scale / 4.0));
		p.setBrush(Qt::NoBrush);
	}

	// --- goal ring (green) ---
	if(snap->hasGoal)
	{
		QPen pen(QColor(50, 230, 50));
		pen.setWidthF(2.0);
		p.setPen(pen);
		double rr = std::max(5.0, scale / 3.0);
		p.drawEllipse(toScreen(snap->goalCell.first, snap->goalCell.second), rr, rr);
	}

	// --- robot pose (blue) dot + heading line ---
	if(snap->hasStart)
	{
		QPointF a = toScreen(sc->first, sc->second);
		double rr = std::max(4.0, scale / 3.0);
		p.setPen(Qt::NoPen);
		p.setBrush(QColor(60, 120, 255));
		p.drawEllipse(a, rr, rr);
		double ln = std::max(scale, 12.0);
		// screen y grows downward -> negate sin (same as the pygame viewer)
		QPointF tip(a.x() + ln * std::cos(snap->meta.yaw), a.y() - ln * std::sin(snap->meta.yaw));
		QPen pen(QColor(60, 120, 255));
		pen.setWidthF(std::max(2.0, scale / 12.0));
		p.setPen(pen);
		p.drawLine(a, tip);
		p.setBrush(Qt::NoBrush);
	}

	// --- info overlay (top-left translucent box) ---
	const char * goalLabel = snap->goalSrc == PathGenerator::GOAL_EXIT   ? "exit tag (10)" :
	                         snap->goalSrc == PathGenerator::GOAL_MANUAL ? "manual (click)" : "none";
	QStringList lines;
	lines << QString("grid %1x%2  cell %3 m").arg(w).arg(h).arg(snap->meta.cell, 0, 'f', 3);
	lines << QString("pose (%1, %2)  yaw %3")
	             .arg(snap->meta.px, 0, 'f', 2).arg(snap->meta.py, 0, 'f', 2)
	             .arg(snap->meta.yaw * (180.0 / 3.14159265358979323846), 0, 'f', 1);
	lines << QString("goal: %1   path: %2 pts").arg(goalLabel).arg((int)snap->pathCells.size());
	lines << QString("obstacle %1  semantic %2  exit %3")
	             .arg(snap->nObstacle).arg(snap->nSemantic).arg(snap->nExit);
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

	std::shared_ptr<Snapshot> snap;
	{
		std::lock_guard<std::mutex> lk(outMutex_);
		snap = snapshot_;
	}
	if(!snap || snap->grid.empty())
		return;

	const int w = snap->meta.w, h = snap->meta.h;
	double ox, oy;
	double scale = scaleFor(w, h, ox, oy);
	int col = (int)((event->x() - ox) / scale);
	int row = (int)((event->y() - oy) / scale);     // flipped/north-up row
	if(col < 0 || col >= w || row < 0 || row >= h)
		return;

	// cell -> world (inverse of the planner's world_to_cell)
	float wx = snap->meta.xMin + (col + 0.5f) * snap->meta.cell;
	float wy = snap->meta.yMin + ((h - 1 - row) + 0.5f) * snap->meta.cell;

	{
		std::lock_guard<std::mutex> lk(inMutex_);
		hasManualGoal_ = true;
		manualGoalX_ = wx;
		manualGoalY_ = wy;
		if(haveLast_)
		{
			pendingGrid_ = lastGrid_;
			pendingMeta_ = lastMeta_;
			pendingDirty_ = true;
		}
	}
	inCv_.notify_one();
}

void MapPathView::keyPressEvent(QKeyEvent * event)
{
	if(event->key() == Qt::Key_R)
	{
		resetRequested_ = true;
		{
			std::lock_guard<std::mutex> lk(inMutex_);
			if(haveLast_)
			{
				pendingGrid_ = lastGrid_;
				pendingMeta_ = lastMeta_;
				pendingDirty_ = true;
			}
		}
		inCv_.notify_one();
	}
	else
	{
		QWidget::keyPressEvent(event);
	}
}

} // namespace rtabmap
