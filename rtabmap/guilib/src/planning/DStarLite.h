/*
 * DStarLite.h - C++ port of the project's Python D* Lite path planner.
 *
 * Faithful translation of path_planning/dstar_core3.py (the @njit core) and the
 * PathGenerator from path_planning/prototype_prd3.py. The algorithm and all of
 * its hyperparameters are kept identical to the Python prototype; only the host
 * language changed (numba -> native C++).
 *
 * Pure C++/OpenCV — no Qt — so it can run on a background worker thread inside
 * the GUI (see MapPathView).
 *
 * Coordinate convention (matches the Python prototype):
 *   - Input cv::Mat is CV_8SC1 with row 0 on the yMin side (as RTABMap streams it).
 *   - Internally the grid is flipped vertically (np.flipud) so row 0 = north (max y).
 *   - All reported cells (start/goal/path) are in this flipped, north-up frame,
 *     which is also the on-screen frame used by MapPathView's painter.
 */
#ifndef RTABMAP_GUI_DSTARLITE_H_
#define RTABMAP_GUI_DSTARLITE_H_

#include <vector>
#include <utility>
#include <cstdint>
#include <opencv2/core.hpp>

namespace rtabmap {

// Grid metadata mirroring the RTABMap TCP wire header (GridTcpStreamer).
struct GridMeta
{
	int   w   = 0;
	int   h   = 0;
	float xMin = 0.f;
	float yMin = 0.f;
	float cell = 0.05f;
	float px  = 0.f;   // robot pose x (m)
	float py  = 0.f;   // robot pose y (m)
	float yaw = 0.f;   // robot heading (rad)
};

// RTABMap occupancy cell values (int8 wire legend, see grid_client.py).
enum RtabCell
{
	kCellUnknown      = -1,
	kCellFree         = 0,
	kCellSemanticWall = 1,
	kCellExit         = 10,
	kCellObstacle     = 100,
};

class PathGenerator
{
public:
	enum GoalSrc { GOAL_NONE = 0, GOAL_EXIT = 1, GOAL_MANUAL = 2 };

	PathGenerator();

	// Compute the pose->exit path for one occupancy-grid frame.
	//   rawI8     : CV_8SC1 grid, shape (meta.h, meta.w), row 0 on the yMin side.
	//   goalWorld : optional manual goal (world metres), used only as a fallback
	//               when the map has no exit tag (cell value 10).
	// Returns world waypoints [(wx,wy), ...] in metres; empty when there is no goal.
	std::vector<std::pair<float, float> > compute(
			const GridMeta & meta,
			const cv::Mat & rawI8,
			bool hasGoalWorld = false,
			float goalWorldX = 0.f,
			float goalWorldY = 0.f);

	// Last-compute results, in the flipped/north-up cell frame (what the painter uses).
	const std::vector<std::pair<int, int> > & pathCells() const { return pathCells_; }
	bool hasStart() const { return hasStart_; }
	bool hasGoal()  const { return hasGoal_; }
	std::pair<int, int> startCell() const { return startCell_; }
	std::pair<int, int> goalCell()  const { return goalCell_; }
	GoalSrc goalSrc() const { return goalSrc_; }
	int gridWidth()  const { return gw_; }
	int gridHeight() const { return gh_; }

	void reset();   // force a clean full replan on the next compute()

private:
	void ensure(int w, int h);
	std::vector<std::pair<int, int> > extractPath(int sx, int sy, int gx, int gy,
	                                              double dMax, double dMaxUnk) const;

	// Plan one uniform-resolution frame (any cell size) to an explicit world goal,
	// reusing the full D* Lite pipeline. Fills outCellPath (this grid's flipped frame)
	// when non-null. Used for both the coarse global pass and the fine local window.
	std::vector<std::pair<float, float> > planFrame(
			const GridMeta & meta, const cv::Mat & rawI8,
			float goalWx, float goalWy,
			std::vector<std::pair<int, int> > * outCellPath = 0);

	int gw_ = 0, gh_ = 0;
	std::vector<double> gMap_;        // g values            [y*w + x]
	std::vector<double> rhsMap_;      // rhs values          [y*w + x]
	std::vector<int>    localGrid_;   // planner legend      [y*w + x]
	std::vector<double> distWallMap_; // distance-to-wall    [y*w + x]
	std::vector<double> distUnknownMap_; // distance-to-unknown [y*w + x]

	std::vector<std::pair<int, int> > pathCells_;
	std::pair<int, int> startCell_{0, 0};
	std::pair<int, int> goalCell_{0, 0};
	bool    hasStart_ = false;
	bool    hasGoal_  = false;
	GoalSrc goalSrc_  = GOAL_NONE;
};

} // namespace rtabmap

#endif // RTABMAP_GUI_DSTARLITE_H_
