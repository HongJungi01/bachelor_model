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
#include <QElapsedTimer>
#include <QImage>

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
	// Only stores the latest grid; the D* replan runs on a fixed 2 s cadence on the
	// worker (not on every grid frame) — see workerLoop().
	void updateGrid(const cv::Mat & map8S, float xMin, float yMin, float cellSize,
	                float poseX, float poseY, float poseYaw);

	// Feed the latest robot pose (map frame) at odometry rate, decoupled from the
	// grid/D* cadence. Updates the live pose marker in real time so it never freezes
	// waiting on a D* replan. Cheap: stores the pose and repaints (throttled).
	void updatePose(float poseX, float poseY, float poseYaw);

public:
	// Last planned path (world metres) + current live pose, returned together under
	// lock. Groundwork for path-following (pure-pursuit) — returns false until a path
	// and a pose are both available. Steering itself is not computed here.
	bool latestPlanAndPose(std::vector<std::pair<float, float> > & worldPath,
	                       float & px, float & py, float & yaw) const;

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
		std::vector<std::pair<float, float> > worldPath; // world metres (for steering)
		std::pair<int, int> startCell{0, 0};
		std::pair<int, int> goalCell{0, 0};
		float goalX = 0.f, goalY = 0.f; // goal in world metres (drawn against the live grid)
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
	std::atomic<bool> stop_{false};
	std::atomic<bool> resetRequested_{false};
	// Wake the worker for an immediate replan (first grid, manual goal, 'R' reset).
	// Otherwise the worker replans on its own fixed 2 s timed wait.
	std::atomic<bool> forceReplan_{false};

	// manual goal fallback (guarded by inMutex_)
	bool  hasManualGoal_ = false;
	float manualGoalX_ = 0.f, manualGoalY_ = 0.f;

	// latest grid frame (guarded by inMutex_); the worker plans on whatever is here
	// at each 2 s tick, and click / 'R' re-trigger planning on it.
	cv::Mat  lastGrid_;
	GridMeta lastMeta_;
	bool     haveLast_ = false;

	mutable std::mutex outMutex_;
	std::shared_ptr<Snapshot> snapshot_;

	// live robot pose (map frame), fed at odometry rate and painted in real time,
	// independent of the D* snapshot. Guarded by poseMutex_.
	mutable std::mutex poseMutex_;
	float liveX_ = 0.f, liveY_ = 0.f, liveYaw_ = 0.f;
	bool  havePose_ = false;
	QElapsedTimer repaintThrottle_;   // cap pose/grid-driven repaints (~30 Hz)

	// realtime display grid (GUI-thread only: written by updateGrid, read by
	// paintEvent/mousePressEvent). Painted immediately, independent of the async D*
	// snapshot, so map + pose stay real-time and only the path waits on the plan.
	cv::Mat  dispGrid_;
	GridMeta dispMeta_;
	bool     haveDisp_ = false;
	unsigned long dispSeq_ = 0;       // bumped each grid frame; keys the map-image cache

	// map-image cache (GUI thread only): rebuilt only when the display grid changes,
	// so pose-rate repaints don't re-run the per-cell grid->QImage loop every frame.
	QImage      mapImage_;
	unsigned long lastDispSeq_ = (unsigned long)-1;
};

} // namespace rtabmap

#endif // RTABMAP_GUI_MAPPATHVIEW_H_
