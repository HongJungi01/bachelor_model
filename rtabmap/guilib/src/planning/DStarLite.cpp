/*
 * DStarLite.cpp - C++ port of path_planning/dstar_core3.py + PathGenerator.
 *
 * The numerics are a 1:1 translation of the Python prototype. Where the Python
 * relied on numba @njit kernels, this uses plain loops; the math, the cost
 * hyperparameters and the control flow are unchanged so the produced paths match.
 */
#include "planning/DStarLite.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace rtabmap {

namespace {

// --- hyperparameters (identical to dstar_core3.py) -------------------------
const double COST_FREE            = 10.0;
const double COST_DIAG            = 14.0;
const double COST_OBSTACLE        = 10000.0;
const double COST_UNEXPLORED      = 50.0;
const double COST_UNEXPLORED_DIAG = 70.7;

const double D_MAX              = 8.0;   // wall clearance band (cells; ~0.4 m @ 0.05)
const double W_MARGIN           = 50.0;  // wall margin weight (graded toward centreline)
// Separate, gentler clearance from unexplored space: nudges the path off the unknown
// frontier without the strong push that would block goal/frontier progress.
const double D_MAX_UNKNOWN      = 5.0;   // unknown clearance band (cells; ~0.25 m @ 0.05)
const double W_MARGIN_UNKNOWN   = 20.0;  // unknown margin weight (< W_MARGIN)
const int    TEMPORAL_THRESHOLD = 3;

const int    HINT_DEPTH_K   = 4;
const int    HINT_WIDTH_L   = 2;
const double HINT_BONUS_VAL = 50.0;
const double FAKE_PENALTY_VAL = 10.0;

// planner legend (to_planner_grid output)
const int P_FREE     = 0;
const int P_OBSTACLE = 100;
const int P_UNKNOWN  = 255;

const double INF = std::numeric_limits<double>::infinity();

// Direction look-up tables for codes 11..18 / 21..28 / 31..38 (8-neighbour dirs).
struct DirLut
{
	int x[256];
	int y[256];
	DirLut()
	{
		for(int i=0; i<256; ++i) { x[i] = 0; y[i] = 0; }
		const int dx[8] = { 0,  1, 1, 1, 0, -1, -1, -1};
		const int dy[8] = {-1, -1, 0, 1, 1,  1,  0, -1};
		for(int i=0; i<8; ++i)
		{
			x[11+i] = dx[i]; y[11+i] = dy[i];
			x[21+i] = dx[i]; y[21+i] = dy[i];
			x[31+i] = dx[i]; y[31+i] = dy[i];
		}
	}
};
const DirLut DIR_LUT;

// to_planner_grid(): RTABMap int8 -> planner legend. Everything except
// unknown(-1)/semantic-wall(1)/obstacle(100) is free(0); exit(10) and the
// reserved direction tags stay free here (goal handled separately).
inline int toPlanner(signed char raw)
{
	if(raw == kCellUnknown)      return P_UNKNOWN;
	if(raw == kCellSemanticWall) return P_OBSTACLE;
	if(raw == kCellObstacle)     return P_OBSTACLE;
	return P_FREE;
}

inline int clampi(int v, int lo, int hi)
{
	return v < lo ? lo : (v > hi ? hi : v);
}

// Min-heap key, ordered like the Python heapq tuple (k1, k2, x, y).
struct Key { double k1, k2; int x, y; };
struct KeyGreater
{
	bool operator()(const Key & a, const Key & b) const
	{
		if(a.k1 != b.k1) return a.k1 > b.k1;
		if(a.k2 != b.k2) return a.k2 > b.k2;
		if(a.x  != b.x ) return a.x  > b.x;
		return a.y > b.y;
	}
};
typedef std::priority_queue<Key, std::vector<Key>, KeyGreater> PQ;

inline double heuristic(int ux, int uy, int sx, int sy)
{
	return (double)std::max(std::abs(ux - sx), std::abs(uy - sy));
}

} // namespace

PathGenerator::PathGenerator() {}

void PathGenerator::reset()
{
	// Drop sizing so the next compute() re-initialises everything (== Python
	// reconstructing PathGenerator). reset_and_replan also fills g/rhs anew.
	gw_ = gh_ = 0;
	gMap_.clear(); rhsMap_.clear(); localGrid_.clear();
	distWallMap_.clear(); distUnknownMap_.clear();
	pathCells_.clear();
	hasStart_ = hasGoal_ = false;
	goalSrc_ = GOAL_NONE;
}

void PathGenerator::ensure(int w, int h)
{
	if(w == gw_ && h == gh_)
		return;
	gw_ = w; gh_ = h;
	const size_t n = (size_t)w * (size_t)h;
	gMap_.assign(n, INF);
	rhsMap_.assign(n, INF);
	localGrid_.assign(n, P_FREE);
	distWallMap_.assign(n, 30.0);
	distUnknownMap_.assign(n, 30.0);
}

// --- BFS distance map to the nearest cell of `sourceVal` (update_dist_wall_map) ---
// sourceVal = P_OBSTACLE gives distance-to-wall; P_UNKNOWN gives distance-to-unknown.
namespace {
void updateDistMap(const std::vector<int> & lg, std::vector<double> & dw,
                   int w, int h, double maxDist, int sourceVal)
{
	std::queue<std::pair<int,int> > q;
	for(int y=0; y<h; ++y)
		for(int x=0; x<w; ++x)
		{
			if(lg[(size_t)y*w + x] == sourceVal)
			{
				dw[(size_t)y*w + x] = 0.0;
				q.push(std::make_pair(x, y));
			}
			else
			{
				dw[(size_t)y*w + x] = 30.0;
			}
		}

	while(!q.empty())
	{
		int cx = q.front().first;
		int cy = q.front().second;
		q.pop();
		double cd = dw[(size_t)cy*w + cx];
		if(cd >= maxDist)
			continue;
		for(int dx=-1; dx<=1; ++dx)
			for(int dy=-1; dy<=1; ++dy)
			{
				if(dx==0 && dy==0) continue;
				int nx = cx + dx, ny = cy + dy;
				if(nx<0 || nx>=w || ny<0 || ny>=h) continue;
				double nd = cd + std::sqrt((double)(dx*dx + dy*dy));
				if(nd < dw[(size_t)ny*w + nx])
				{
					dw[(size_t)ny*w + nx] = nd;
					q.push(std::make_pair(nx, ny));
				}
			}
	}
}
} // namespace

// --- edge cost (calc_edge_cost) --------------------------------------------
static double calcEdgeCost(int ux, int uy, int vx, int vy,
                           const std::vector<int> & lg, const std::vector<double> & dw,
                           const std::vector<double> & duw,
                           int w, int h, int temporalCounter)
{
	const int pixelVal = lg[(size_t)vy*w + vx];
	const bool isDiag = (ux != vx) && (uy != vy);

	double cBase;
	if(pixelVal == P_OBSTACLE)
		return COST_OBSTACLE;
	else if(pixelVal == P_UNKNOWN)
		cBase = isDiag ? COST_UNEXPLORED_DIAG : COST_UNEXPLORED;
	else
		cBase = isDiag ? COST_DIAG : COST_FREE;

	double cAoe = 0.0;
	const int minY = std::max(0,     vy - HINT_DEPTH_K);
	const int maxY = std::min(h - 1, vy + HINT_DEPTH_K);
	const int minX = std::max(0,     vx - HINT_DEPTH_K);
	const int maxX = std::min(w - 1, vx + HINT_DEPTH_K);
	for(int ny=minY; ny<=maxY; ++ny)
		for(int nx=minX; nx<=maxX; ++nx)
		{
			int val = lg[(size_t)ny*w + nx];
			if((11<=val && val<=18) || (21<=val && val<=28) || (31<=val && val<=38))
			{
				int dx = DIR_LUT.x[val], dy = DIR_LUT.y[val];
				int rx = vx - nx, ry = vy - ny;
				int projFwd = rx*dx + ry*dy;
				int projLat = rx*(-dy) + ry*dx;
				if(projFwd>=1 && projFwd<=HINT_DEPTH_K && std::abs(projLat)<=HINT_WIDTH_L)
				{
					if(11<=val && val<=18)      cAoe -= HINT_BONUS_VAL;
					else if(31<=val && val<=38) cAoe += FAKE_PENALTY_VAL;
				}
			}
		}

	double floorVal = cBase * 0.1;
	cBase = std::max(floorVal, cBase + cAoe);

	double dWall = dw[(size_t)vy*w + vx];
	double cMargin = 0.0;
	if(dWall < D_MAX)
		cMargin = W_MARGIN * (D_MAX - dWall);

	// Gentler clearance from unexplored space (separate radius/weight from walls).
	double dUnk = duw[(size_t)vy*w + vx];
	double cMarginUnk = 0.0;
	if(dUnk < D_MAX_UNKNOWN)
		cMarginUnk = W_MARGIN_UNKNOWN * (D_MAX_UNKNOWN - dUnk);

	double cDir = 0.0;
	if(21<=pixelVal && pixelVal<=28)
	{
		if(temporalCounter >= TEMPORAL_THRESHOLD)
		{
			int adx = DIR_LUT.x[pixelVal], ady = DIR_LUT.y[pixelVal];
			if(((vx-ux)*adx) + ((vy-uy)*ady) < 0)
				cDir = COST_OBSTACLE;
		}
	}

	return cBase + cMargin + cMarginUnk + cDir;
}

// --- D* Lite core ----------------------------------------------------------
namespace {

inline Key calculateKey(int ux, int uy, int sx, int sy,
                        const std::vector<double> & g, const std::vector<double> & rhs,
                        int w, double km)
{
	double minVal = std::min(g[(size_t)uy*w + ux], rhs[(size_t)uy*w + ux]);
	double hVal = heuristic(ux, uy, sx, sy);
	Key k;
	k.k1 = minVal + hVal + km;
	k.k2 = minVal;
	k.x = ux; k.y = uy;
	return k;
}

void updateVertex(int ux, int uy, int sx, int sy, int tx, int ty,
                  std::vector<double> & g, std::vector<double> & rhs,
                  const std::vector<int> & lg, const std::vector<double> & dw,
                  const std::vector<double> & duw,
                  int w, int h, PQ & pq, double km, int temporal)
{
	if(!(ux==tx && uy==ty))
	{
		double minRhs = INF;
		for(int dx=-1; dx<=1; ++dx)
			for(int dy=-1; dy<=1; ++dy)
			{
				if(dx==0 && dy==0) continue;
				int vx = ux+dx, vy = uy+dy;
				if(vx<0 || vx>=w || vy<0 || vy>=h) continue;
				double cost = calcEdgeCost(ux, uy, vx, vy, lg, dw, duw, w, h, temporal);
				double val = cost + g[(size_t)vy*w + vx];
				if(val < minRhs) minRhs = val;
			}
		rhs[(size_t)uy*w + ux] = minRhs;
	}

	if(g[(size_t)uy*w + ux] != rhs[(size_t)uy*w + ux])
		pq.push(calculateKey(ux, uy, sx, sy, g, rhs, w, km));
}

void computeShortestPath(int sx, int sy, int tx, int ty,
                         std::vector<double> & g, std::vector<double> & rhs,
                         const std::vector<int> & lg, const std::vector<double> & dw,
                         const std::vector<double> & duw,
                         int w, int h, PQ & pq, double km, int temporal)
{
	while(!pq.empty())
	{
		Key top = pq.top();
		pq.pop();
		int ux = top.x, uy = top.y;
		double kOld1 = top.k1, kOld2 = top.k2;

		Key kNew = calculateKey(ux, uy, sx, sy, g, rhs, w, km);
		if(kOld1 != kNew.k1 || kOld2 != kNew.k2)
		{
			if(kOld1 < kNew.k1 || (kOld1 == kNew.k1 && kOld2 < kNew.k2))
				pq.push(kNew);
			continue;
		}

		double gVal = g[(size_t)uy*w + ux];
		double rhsVal = rhs[(size_t)uy*w + ux];
		if(gVal == rhsVal)
			continue;

		Key startKey = calculateKey(sx, sy, sx, sy, g, rhs, w, km);
		bool isKOldGreater = (kOld1 > startKey.k1) ||
		                     (kOld1 == startKey.k1 && kOld2 >= startKey.k2);
		if(isKOldGreater && (g[(size_t)sy*w + sx] == rhs[(size_t)sy*w + sx]))
			break;

		if(gVal > rhsVal)
		{
			g[(size_t)uy*w + ux] = rhsVal;
			for(int dx=-1; dx<=1; ++dx)
				for(int dy=-1; dy<=1; ++dy)
				{
					if(dx==0 && dy==0) continue;
					int vx = ux+dx, vy = uy+dy;
					if(vx<0 || vx>=w || vy<0 || vy>=h) continue;
					updateVertex(vx, vy, sx, sy, tx, ty, g, rhs, lg, dw, duw, w, h, pq, km, temporal);
				}
		}
		else
		{
			g[(size_t)uy*w + ux] = INF;
			updateVertex(ux, uy, sx, sy, tx, ty, g, rhs, lg, dw, duw, w, h, pq, km, temporal);
			for(int dx=-1; dx<=1; ++dx)
				for(int dy=-1; dy<=1; ++dy)
				{
					if(dx==0 && dy==0) continue;
					int vx = ux+dx, vy = uy+dy;
					if(vx<0 || vx>=w || vy<0 || vy>=h) continue;
					updateVertex(vx, vy, sx, sy, tx, ty, g, rhs, lg, dw, duw, w, h, pq, km, temporal);
				}
		}
	}
}

} // namespace

// --- greedy path extraction (PathGenerator._extract_path) ------------------
std::vector<std::pair<int,int> > PathGenerator::extractPath(int sx, int sy, int gx, int gy) const
{
	std::vector<std::pair<int,int> > path;
	path.push_back(std::make_pair(sx, sy));
	std::vector<char> seen((size_t)gw_ * gh_, 0);
	seen[(size_t)sy*gw_ + sx] = 1;

	int cx = sx, cy = sy;
	const long maxSteps = (long)gw_ * gh_;
	for(long step=0; step<maxSteps; ++step)
	{
		if(cx==gx && cy==gy)
			break;
		double best = INF;
		int nxBest = cx, nyBest = cy;
		for(int dx=-1; dx<=1; ++dx)
			for(int dy=-1; dy<=1; ++dy)
			{
				if(dx==0 && dy==0) continue;
				int nx = cx+dx, ny = cy+dy;
				if(nx<0 || nx>=gw_ || ny<0 || ny>=gh_) continue;
				double c = calcEdgeCost(cx, cy, nx, ny, localGrid_, distWallMap_, distUnknownMap_, gw_, gh_, 3)
				           + gMap_[(size_t)ny*gw_ + nx];
				if(c < best) { best = c; nxBest = nx; nyBest = ny; }
			}
		if((nxBest==cx && nyBest==cy) || best==INF || seen[(size_t)nyBest*gw_ + nxBest])
			break;
		seen[(size_t)nyBest*gw_ + nxBest] = 1;
		path.push_back(std::make_pair(nxBest, nyBest));
		cx = nxBest; cy = nyBest;
	}
	return path;
}

// --- one-frame compute (PathGenerator.compute) -----------------------------
std::vector<std::pair<float,float> > PathGenerator::compute(
		const GridMeta & meta, const cv::Mat & rawI8,
		bool hasGoalWorld, float goalWorldX, float goalWorldY)
{
	std::vector<std::pair<float,float> > waypoints;
	pathCells_.clear();
	hasStart_ = hasGoal_ = false;
	goalSrc_ = GOAL_NONE;

	const int w = meta.w, h = meta.h;
	if(w <= 0 || h <= 0 || rawI8.empty() || rawI8.rows != h || rawI8.cols != w)
		return waypoints;

	ensure(w, h);

	// Build flipped (north-up) planner grid and locate exit-tag cells in one pass.
	double exitSumX = 0.0, exitSumY = 0.0;
	long exitCount = 0;
	for(int y=0; y<h; ++y)
	{
		const signed char * srcRow = rawI8.ptr<signed char>(h - 1 - y); // flipud
		int * dstRow = &localGrid_[(size_t)y*w];
		for(int x=0; x<w; ++x)
		{
			signed char raw = srcRow[x];
			dstRow[x] = toPlanner(raw);
			if(raw == kCellExit)
			{
				exitSumX += x; exitSumY += y; ++exitCount;
			}
		}
	}

	updateDistMap(localGrid_, distWallMap_,    w, h, D_MAX,         P_OBSTACLE);
	updateDistMap(localGrid_, distUnknownMap_, w, h, D_MAX_UNKNOWN, P_UNKNOWN);

	// Start = robot pose cell (flipped frame).
	auto worldToCell = [&](float wx, float wy, int & col, int & row) {
		col = (int)((wx - meta.xMin) / meta.cell);
		row = (h - 1) - (int)((wy - meta.yMin) / meta.cell);
	};
	int sx, sy;
	worldToCell(meta.px, meta.py, sx, sy);
	sx = clampi(sx, 0, w-1);
	sy = clampi(sy, 0, h-1);
	startCell_ = std::make_pair(sx, sy);
	hasStart_ = true;

	// Goal = exit tag (nearest exit cell to the exit centroid), else manual fallback.
	int gx, gy;
	if(exitCount > 0)
	{
		double cxm = exitSumX / exitCount, cym = exitSumY / exitCount;
		double bestD = INF;
		int bx = -1, by = -1;
		for(int y=0; y<h; ++y)
		{
			const signed char * srcRow = rawI8.ptr<signed char>(h - 1 - y);
			for(int x=0; x<w; ++x)
				if(srcRow[x] == kCellExit)
				{
					double d = (x - cxm)*(x - cxm) + (y - cym)*(y - cym);
					if(d < bestD) { bestD = d; bx = x; by = y; }
				}
		}
		gx = bx; gy = by;
		goalSrc_ = GOAL_EXIT;
	}
	else if(hasGoalWorld)
	{
		worldToCell(goalWorldX, goalWorldY, gx, gy);
		gx = clampi(gx, 0, w-1);
		gy = clampi(gy, 0, h-1);
		goalSrc_ = GOAL_MANUAL;
	}
	else
	{
		return waypoints; // no goal
	}

	goalCell_ = std::make_pair(gx, gy);
	hasGoal_ = true;

	// reset_and_replan(sx,sy,gx,gy, ..., km=0, temporal=3)
	const double km = 0.0;
	const int temporal = TEMPORAL_THRESHOLD;
	std::fill(gMap_.begin(), gMap_.end(), INF);
	std::fill(rhsMap_.begin(), rhsMap_.end(), INF);
	PQ pq;
	rhsMap_[(size_t)gy*w + gx] = 0.0;
	pq.push(calculateKey(gx, gy, sx, sy, gMap_, rhsMap_, w, km));
	computeShortestPath(sx, sy, gx, gy, gMap_, rhsMap_, localGrid_, distWallMap_,
	                    distUnknownMap_, w, h, pq, km, temporal);

	pathCells_ = extractPath(sx, sy, gx, gy);

	// cell_to_world for each path cell.
	waypoints.reserve(pathCells_.size());
	for(size_t i=0; i<pathCells_.size(); ++i)
	{
		int col = pathCells_[i].first, row = pathCells_[i].second;
		float wx = meta.xMin + (col + 0.5f) * meta.cell;
		float wy = meta.yMin + ((h - 1 - row) + 0.5f) * meta.cell;
		waypoints.push_back(std::make_pair(wx, wy));
	}
	return waypoints;
}

} // namespace rtabmap
