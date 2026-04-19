/*
 * GridPublisher.h — rtabmap_unity_bridge edition
 *
 * UEventsHandler that receives RtabmapEvent/OdometryEvent,
 * extracts a 2D occupancy grid, visualizes it with OpenCV,
 * and streams it over TCP to Unity (localhost:7777).
 *
 * Adds processOffline() for batch (non-threaded) pipeline usage.
 *
 * Based on rtabmap_2d_pipeline/GridPublisher.h
 */

#ifndef GRIDPUBLISHER_H_
#define GRIDPUBLISHER_H_

// ── Must come FIRST to prevent winsock.h / winsock2.h conflict ──
#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#endif

#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util3d_mapping.h"
#include "rtabmap/core/RtabmapEvent.h"
#include "rtabmap/core/OdometryEvent.h"
#include "rtabmap/core/global_map/OccupancyGrid.h"
#include "rtabmap/utilite/UEventsHandler.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UTimer.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include <mutex>
#include <atomic>
#include <cmath>
#include <string>
#include <fstream>

// ── TCP headers (platform typedefs) ──────────────────────────
#ifdef _WIN32
typedef SOCKET socket_t;
#define INVALID_SOCK INVALID_SOCKET
#define CLOSE_SOCKET closesocket
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
typedef int socket_t;
#define INVALID_SOCK (-1)
#define CLOSE_SOCKET close
#endif

using namespace rtabmap;

// ═══════════════════════════════════════════════════════════════
//  TCP packet header — 32 bytes, little-endian
// ═══════════════════════════════════════════════════════════════
#pragma pack(push, 1)
struct GridPacketHeader {
	int32_t  width;      // grid columns
	int32_t  height;     // grid rows
	float    xMin;       // world X of grid origin (metres)
	float    yMin;       // world Y of grid origin (metres)
	float    cellSize;   // metres per cell
	float    poseX;      // current robot X (metres)
	float    poseY;      // current robot Y (metres)
	float    poseYaw;    // current robot heading (radians)
};
#pragma pack(pop)
static_assert(sizeof(GridPacketHeader) == 32, "Header must be 32 bytes");


class GridPublisher : public UEventsHandler
{
public:
	GridPublisher(
		int tcpPort   = 7777,
		double minPublishPeriod = 0.1,        // seconds (≈10 Hz cap)
		const std::string & outputDir = "output")
		: grid_(&localGrids_),
		  odometryCorrection_(Transform::getIdentity()),
		  tcpPort_(tcpPort),
		  minPublishPeriod_(minPublishPeriod),
		  outputDir_(outputDir),
		  listenSock_(INVALID_SOCK),
		  clientSock_(INVALID_SOCK),
		  running_(true),
		  frameCount_(0)
	{
		initTcpServer();
		UINFO("GridPublisher: TCP server on port %d, output dir '%s'",
		      tcpPort_, outputDir_.c_str());
	}

	virtual ~GridPublisher()
	{
		running_ = false;
		this->unregisterFromEventsManager();
		closeTcp();
		cv::destroyAllWindows();
	}

	void setRunning(bool v) { running_ = v; }
	bool isRunning() const  { return running_; }

	// ────────────────────────────────────────────────────────────
	//  Offline mode: direct call (no event system required)
	//
	//  Call this from a batch loop after rtabmap.process() succeeds.
	//  rawOdomPose = the odometry pose returned by odom->process().
	// ────────────────────────────────────────────────────────────
	void processOffline(const Statistics & stats, const Transform & rawOdomPose)
	{
		// Compute corrected pose (normally done by processOdometry)
		Transform corrected = rawOdomPose;
		if (!stats.mapCorrection().isNull())
			corrected = stats.mapCorrection() * rawOdomPose;
		{
			std::lock_guard<std::mutex> lk(poseMtx_);
			currentPose_ = corrected;
		}
		processStatistics(stats);
	}

	// Save final map regardless of frame count
	void saveFinalMap()
	{
		float xMin = 0.f, yMin = 0.f;
		cv::Mat map8S = grid_.getMap(xMin, yMin);
		if (!map8S.empty())
			saveMapFile(map8S, xMin, yMin, grid_.getCellSize());
	}

protected:
	// ────────────────────────────────────────────────────────────
	//  Event dispatcher (called from RTAB-Map threads)
	// ────────────────────────────────────────────────────────────
	virtual bool handleEvent(UEvent * event) override
	{
		if (!running_) return false;

		if (event->getClassName().compare("RtabmapEvent") == 0)
		{
			RtabmapEvent * e = static_cast<RtabmapEvent *>(event);
			processStatistics(e->getStats());
		}
		else if (event->getClassName().compare("OdometryEvent") == 0)
		{
			OdometryEvent * e = static_cast<OdometryEvent *>(event);
			processOdometry(*e);
		}
		return false;
	}

private:
	// ────────────────────────────────────────────────────────────
	//  Odometry → update current pose
	// ────────────────────────────────────────────────────────────
	void processOdometry(const OdometryEvent & odom)
	{
		if (odom.pose().isNull()) return;
		std::lock_guard<std::mutex> lk(poseMtx_);
		currentPose_ = odometryCorrection_ * odom.pose();
	}

	// ────────────────────────────────────────────────────────────
	//  RtabmapEvent → extract grid → visualise + publish
	// ────────────────────────────────────────────────────────────
	void processStatistics(const Statistics & stats)
	{
		// ── rate limiting ──
		if (timer_.elapsed() < minPublishPeriod_) return;
		timer_.restart();

		// ── 1. Feed local grid cache ──
		int lastId = stats.getLastSignatureData().id();
		if (grid_.addedNodes().find(lastId) == grid_.addedNodes().end())
		{
			float cellSize = stats.getLastSignatureData().sensorData().gridCellSize();
			if (cellSize > 0.0f)
			{
				cv::Mat ground, obstacles, empty;
				stats.getLastSignatureData().sensorData()
					.uncompressDataConst(0, 0, 0, 0, &ground, &obstacles, &empty);
				localGrids_.add(
					lastId, ground, obstacles, empty,
					cellSize,
					stats.getLastSignatureData().sensorData().gridViewPoint());
			}
		}

		// ── 2. Assemble global grid ──
		if (grid_.addedNodes().size() || localGrids_.size())
			grid_.update(stats.poses());

		if (grid_.addedNodes().empty())
			return;

		// ── 3. Extract map ──
		float xMin = 0.f, yMin = 0.f;
		cv::Mat map8S = grid_.getMap(xMin, yMin);
		if (map8S.empty()) return;

		float cellSize = grid_.getCellSize();

		// ── Update odometry correction ──
		odometryCorrection_ = stats.mapCorrection();

		// ── Snapshot pose ──
		float poseX, poseY, poseYaw;
		{
			std::lock_guard<std::mutex> lk(poseMtx_);
			if (currentPose_.isNull()) return;
			poseX = currentPose_.x();
			poseY = currentPose_.y();
			float r, p, y;
			currentPose_.getEulerAngles(r, p, y);
			poseYaw = y;
		}

		// ── 4. Visualise with OpenCV ──
		visualise(map8S, xMin, yMin, cellSize, poseX, poseY, poseYaw);

		// ── 5. Send over TCP ──
		sendTcp(map8S, xMin, yMin, cellSize, poseX, poseY, poseYaw);

		// ── 6. Periodic file save (every 50 frames) ──
		if (++frameCount_ % 50 == 0)
			saveMapFile(map8S, xMin, yMin, cellSize);
	}

	// ────────────────────────────────────────────────────────────
	//  OpenCV 2D visualisation
	// ────────────────────────────────────────────────────────────
	void visualise(const cv::Mat & map8S,
	               float xMin, float yMin, float cellSize,
	               float poseX, float poseY, float poseYaw)
	{
		// Convert signed occupancy to unsigned grayscale
		cv::Mat gray = util3d::convertMap2Image8U(map8S);
		cv::Mat color;
		cv::cvtColor(gray, color, cv::COLOR_GRAY2BGR);

		// Robot position in pixel coordinates
		int px = static_cast<int>((poseX - xMin) / cellSize);
		int py = static_cast<int>(color.rows - 1 - (poseY - yMin) / cellSize);

		if (px >= 0 && px < color.cols && py >= 0 && py < color.rows)
		{
			// Red dot for current position
			cv::circle(color, cv::Point(px, py), 5, cv::Scalar(0, 0, 255), -1);

			// Arrow showing heading direction
			int arrowLen = 20;
			int dx = static_cast<int>(arrowLen * std::cos(poseYaw));
			int dy = static_cast<int>(-arrowLen * std::sin(poseYaw)); // y-axis inverted
			cv::arrowedLine(color, cv::Point(px, py),
			                cv::Point(px + dx, py + dy),
			                cv::Scalar(0, 0, 255), 2, cv::LINE_AA, 0, 0.3);
		}

		// HUD text
		std::string info = cv::format("Grid %dx%d  cell=%.2fm  pose=(%.2f,%.2f)  frame=%d",
		                              map8S.cols, map8S.rows, cellSize, poseX, poseY, frameCount_);
		cv::putText(color, info, cv::Point(10, 20),
		            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);

		cv::imshow("2D Occupancy Grid", color);
		cv::waitKey(1);   // non-blocking
	}

	// ────────────────────────────────────────────────────────────
	//  TCP server — single-client, non-blocking accept
	// ────────────────────────────────────────────────────────────
	void initTcpServer()
	{
#ifdef _WIN32
		WSADATA wsa;
		WSAStartup(MAKEWORD(2, 2), &wsa);
#endif
		listenSock_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (listenSock_ == INVALID_SOCK)
		{
			UERROR("GridPublisher: socket() failed");
			return;
		}

		// Allow port reuse
		int opt = 1;
		setsockopt(listenSock_, SOL_SOCKET, SO_REUSEADDR,
		           reinterpret_cast<const char *>(&opt), sizeof(opt));

		// Set listen socket to non-blocking
#ifdef _WIN32
		u_long mode = 1;
		ioctlsocket(listenSock_, FIONBIO, &mode);
#else
		int flags = fcntl(listenSock_, F_GETFL, 0);
		fcntl(listenSock_, F_SETFL, flags | O_NONBLOCK);
#endif

		struct sockaddr_in addr{};
		addr.sin_family      = AF_INET;
		addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);  // 127.0.0.1 only
		addr.sin_port        = htons(static_cast<uint16_t>(tcpPort_));

		if (bind(listenSock_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) != 0)
		{
			UERROR("GridPublisher: bind() failed on port %d", tcpPort_);
			CLOSE_SOCKET(listenSock_);
			listenSock_ = INVALID_SOCK;
			return;
		}
		listen(listenSock_, 1);
		UINFO("GridPublisher: listening on 127.0.0.1:%d", tcpPort_);
	}

	void tryAccept()
	{
		if (listenSock_ == INVALID_SOCK) return;
		if (clientSock_ != INVALID_SOCK) return;  // already connected

		struct sockaddr_in caddr{};
		socklen_t clen = sizeof(caddr);
		socket_t s = accept(listenSock_, reinterpret_cast<struct sockaddr *>(&caddr), &clen);
		if (s != INVALID_SOCK)
		{
			clientSock_ = s;
			UINFO("GridPublisher: client connected");
		}
	}

	void sendTcp(const cv::Mat & map8S,
	             float xMin, float yMin, float cellSize,
	             float poseX, float poseY, float poseYaw)
	{
		tryAccept();
		if (clientSock_ == INVALID_SOCK) return;

		GridPacketHeader hdr{};
		hdr.width    = map8S.cols;
		hdr.height   = map8S.rows;
		hdr.xMin     = xMin;
		hdr.yMin     = yMin;
		hdr.cellSize = cellSize;
		hdr.poseX    = poseX;
		hdr.poseY    = poseY;
		hdr.poseYaw  = poseYaw;

		// Build contiguous buffer: header + body
		size_t bodySize = static_cast<size_t>(map8S.cols) * map8S.rows;
		std::vector<char> buf(sizeof(hdr) + bodySize);
		std::memcpy(buf.data(), &hdr, sizeof(hdr));

		// Copy map data row-by-row (in case mat is not contiguous)
		if (map8S.isContinuous())
		{
			std::memcpy(buf.data() + sizeof(hdr), map8S.data, bodySize);
		}
		else
		{
			char * dst = buf.data() + sizeof(hdr);
			for (int r = 0; r < map8S.rows; ++r)
			{
				std::memcpy(dst, map8S.ptr<char>(r), map8S.cols);
				dst += map8S.cols;
			}
		}

		int sent = send(clientSock_, buf.data(), static_cast<int>(buf.size()), 0);
		if (sent <= 0)
		{
			UWARN("GridPublisher: client disconnected");
			CLOSE_SOCKET(clientSock_);
			clientSock_ = INVALID_SOCK;
		}
	}

	void closeTcp()
	{
		if (clientSock_ != INVALID_SOCK) { CLOSE_SOCKET(clientSock_); clientSock_ = INVALID_SOCK; }
		if (listenSock_ != INVALID_SOCK) { CLOSE_SOCKET(listenSock_); listenSock_ = INVALID_SOCK; }
#ifdef _WIN32
		WSACleanup();
#endif
	}

	// ────────────────────────────────────────────────────────────
	//  File output (PGM + YAML)
	// ────────────────────────────────────────────────────────────
	void saveMapFile(const cv::Mat & map8S,
	                 float xMin, float yMin, float cellSize)
	{
		if (outputDir_.empty()) return;

		// Create output directory if needed
#ifdef _WIN32
		CreateDirectoryA(outputDir_.c_str(), NULL);
#else
		mkdir(outputDir_.c_str(), 0755);
#endif

		cv::Mat pgm = util3d::convertMap2Image8U(map8S, true);  // PGM format
		std::string pgmPath = outputDir_ + "/map.pgm";
		cv::imwrite(pgmPath, pgm);

		std::string yamlPath = outputDir_ + "/map.yaml";
		std::ofstream f(yamlPath);
		if (f.is_open())
		{
			f << "image: map.pgm\n"
			  << "resolution: " << cellSize << "\n"
			  << "origin: [" << xMin << ", " << yMin << ", 0.0]\n"
			  << "negate: 0\n"
			  << "occupied_thresh: 0.65\n"
			  << "free_thresh: 0.196\n";
		}
		UINFO("GridPublisher: saved %s (%dx%d)", pgmPath.c_str(), map8S.cols, map8S.rows);
	}

	// ── member variables ──
	LocalGridCache  localGrids_;
	OccupancyGrid   grid_;
	Transform       odometryCorrection_;

	std::mutex      poseMtx_;
	Transform       currentPose_;

	int             tcpPort_;
	double          minPublishPeriod_;
	std::string     outputDir_;

	socket_t        listenSock_;
	socket_t        clientSock_;
	std::atomic<bool> running_;

	UTimer          timer_;
	int             frameCount_;
};

#endif /* GRIDPUBLISHER_H_ */
