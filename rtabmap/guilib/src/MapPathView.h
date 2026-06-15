/*
 * MapPathView.h - "2D map + planned path" dock (quadrant 3).
 *
 * Native C++ replacement for the project's Python grid_client.py +
 * path_planning/prototype_prd3.py viewer. It receives the exact occupancy grid +
 * robot pose that MainWindow streams over TCP (no loopback socket — it is fed
 * directly in MainWindow::updateMapCloud), runs the ported D* Lite planner on a
 * background thread, and paints the map, the planned path, the goal and the robot
 * pose with QPainter.
 *
 * Private guilib widget (header lives in src/, not exported); created purely in
 * C++ by MainWindow, mirroring the Semantic dock.
 */
#ifndef RTABMAP_GUI_MAPPATHVIEW_H_
#define RTABMAP_GUI_MAPPATHVIEW_H_

#include <QWidget>

#include <opencv2/core.hpp>

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include "planning/DStarLite.h"

namespace rtabmap {

class MapPathView : public QWidget
{
	Q_OBJECT

public:
	explicit MapPathView(QWidget * parent = 0);
	virtual ~MapPathView();

public Q_SLOTS:
	// Feed one occupancy-grid frame. Same payload GridTcpStreamer::sendGrid packs:
	// map8S is CV_8SC1 (row 0 on the yMin side); xMin/yMin/cellSize are the grid
	// origin/resolution; poseX/Y/Yaw is the latest robot odometry pose.
	void updateGrid(const cv::Mat & map8S, float xMin, float yMin, float cellSize,
	                float poseX, float poseY, float poseYaw);

protected:
	void paintEvent(QPaintEvent * event);
	void mousePressEvent(QMouseEvent * event);
	void keyPressEvent(QKeyEvent * event);

private:
	struct Snapshot
	{
		cv::Mat  grid;        // CV_8SC1, as received (row 0 = yMin side)
		GridMeta meta;
		std::vector<std::pair<int, int> > pathCells; // flipped/north-up cell frame
		std::pair<int, int> startCell{0, 0};
		std::pair<int, int> goalCell{0, 0};
		bool hasStart = false;
		bool hasGoal  = false;
		int  goalSrc  = 0;    // PathGenerator::GoalSrc
		long nUnknown = 0, nFree = 0, nObstacle = 0, nSemantic = 0, nExit = 0;
	};

	void workerLoop();
	// Map widget pixels: returns px/cell and fills the centring offsets, for a grid
	// of w x h cells fit into the current widget rect.
	double scaleFor(int w, int h, double & ox, double & oy) const;

	std::thread worker_;
	std::mutex  inMutex_;
	std::condition_variable inCv_;
	cv::Mat     pendingGrid_;
	GridMeta    pendingMeta_;
	bool        pendingDirty_ = false;
	std::atomic<bool> stop_{false};
	std::atomic<bool> resetRequested_{false};

	// manual goal fallback (guarded by inMutex_)
	bool  hasManualGoal_ = false;
	float manualGoalX_ = 0.f, manualGoalY_ = 0.f;

	// last frame kept on the GUI thread, to re-trigger planning on click / reset
	cv::Mat  lastGrid_;
	GridMeta lastMeta_;
	bool     haveLast_ = false;

	std::mutex outMutex_;
	std::shared_ptr<Snapshot> snapshot_;
};

} // namespace rtabmap

#endif // RTABMAP_GUI_MAPPATHVIEW_H_
