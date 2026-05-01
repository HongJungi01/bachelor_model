/*
 * CameraUnityTCP.h
 *
 * rtabmap::Camera subclass that receives RGB+Depth+IMU frames from Unity
 * over a TCP socket. Acts as a TCP server (Unity is the client).
 *
 * Protocol (little-endian):
 *   Common header: [type:u8][payloadSize:u32]                       = 5B
 *
 *   Type=1 Calib (sent once after connect):
 *     [width:u32][height:u32]
 *     [fx:f64][fy:f64][cx:f64][cy:f64]
 *     [localTransform[12]:f64]   (3x4 row-major, optical → base_link)
 *
 *   Type=2 IMU:
 *     [stamp:f64][gx,gy,gz:f64][ax,ay,az:f64]                       = 56B
 *
 *   Type=3 RGBD:
 *     [stamp:f64][width:u32][height:u32]
 *     [rgb: width*height*3 bytes]   (RGB24, top-left origin)
 *     [depth: width*height*2 bytes] (uint16 mm little-endian, top-left origin)
 */

#ifndef CAMERAUNITYTCP_H_
#define CAMERAUNITYTCP_H_

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
typedef SOCKET socket_t_unity;
#define INVALID_UNITY_SOCK INVALID_SOCKET
#define CLOSE_UNITY_SOCK closesocket
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
typedef int socket_t_unity;
#define INVALID_UNITY_SOCK (-1)
#define CLOSE_UNITY_SOCK close
#endif

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rtabmap/core/Camera.h>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/IMU.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/utilite/ULogger.h>

namespace rtabmap {

// Renamed from CameraUnityTCP to avoid ODR conflict with rtabmap::CameraUnityTCP
// (added to rtabmap core 2026-04-26). Both classes coexist in namespace rtabmap;
// main.cpp uses this local one because rtabmap's lacks the drainIMU(maxStamp,
// sink) lambda-pull pattern this pipeline relies on.
class UnityTcpCameraLocal : public Camera
{
public:
	UnityTcpCameraLocal(int listenPort = 7778, float imageRate = 0.0f)
		: Camera(imageRate),
		  listenPort_(listenPort),
		  listenSock_(INVALID_UNITY_SOCK),
		  clientSock_(INVALID_UNITY_SOCK),
		  running_(false),
		  calibrated_(false),
		  width_(0), height_(0)
	{}

	virtual ~UnityTcpCameraLocal()
	{
		stop();
	}

	// ── Camera interface ─────────────────────────────────────────
	virtual bool init(const std::string & /*calibrationFolder*/ = ".",
	                  const std::string & /*cameraName*/        = "") override
	{
		if (!startServer()) return false;
		running_ = true;
		recvThread_ = std::thread(&UnityTcpCameraLocal::recvLoop, this);

		// Wait for the first calibration packet (max 30s)
		UINFO("CameraUnityTCP: waiting for Unity to connect on port %d...", listenPort_);
		std::unique_lock<std::mutex> lk(calibMtx_);
		bool ok = calibCv_.wait_for(lk, std::chrono::seconds(30),
		                            [this]{ return calibrated_.load() || !running_.load(); });
		if (!calibrated_)
		{
			UERROR("CameraUnityTCP: timed out waiting for calibration packet");
			return false;
		}
		UINFO("CameraUnityTCP: calibration received (%dx%d, fx=%.2f fy=%.2f)",
		      width_, height_, model_.fx(), model_.fy());
		return true;
	}

	virtual bool isCalibrated() const override { return calibrated_; }
	virtual std::string getSerial() const override { return "unity_tcp"; }

	// ── IMU drainage (called by main loop before each image) ─────
	// Pulls every IMU sample with stamp <= maxStamp from the internal queue.
	void drainIMU(double maxStamp, const std::function<void(const IMU &, double)> & sink)
	{
		std::lock_guard<std::mutex> lk(imuMtx_);
		while (!imuQueue_.empty() && imuQueue_.front().stamp <= maxStamp)
		{
			const ImuSample & s = imuQueue_.front();
			IMU imu(cv::Vec3d(s.gx, s.gy, s.gz),
			        cv::Mat::zeros(3, 3, CV_64FC1),
			        cv::Vec3d(s.ax, s.ay, s.az),
			        cv::Mat::zeros(3, 3, CV_64FC1),
			        baseToImu_);
			sink(imu, s.stamp);
			imuQueue_.pop_front();
		}
	}

	void stop()
	{
		running_ = false;
		frameCv_.notify_all();
		calibCv_.notify_all();
		closeClient();
		closeServer();
		if (recvThread_.joinable()) recvThread_.join();
	}

protected:
	virtual SensorData captureImage(SensorCaptureInfo * /*info*/ = 0) override
	{
		std::unique_lock<std::mutex> lk(frameMtx_);
		frameCv_.wait(lk, [this]{ return !frameQueue_.empty() || !running_; });
		if (!running_ || frameQueue_.empty()) return SensorData();

		Frame f = std::move(frameQueue_.front());
		frameQueue_.pop_front();
		lk.unlock();

		// Build cv::Mat that owns its memory (move bytes into Mat clones)
		cv::Mat rgbView(f.height, f.width, CV_8UC3, f.rgb.data());
		cv::Mat rgb;
		cv::cvtColor(rgbView, rgb, cv::COLOR_RGB2BGR);   // OpenCV expects BGR

		cv::Mat depthView(f.height, f.width, CV_16UC1, f.depth.data());
		cv::Mat depth = depthView.clone();

		SensorData data(rgb, depth, model_, getNextSeqID(), f.stamp);
		return data;
	}

private:
	struct ImuSample { double stamp, gx, gy, gz, ax, ay, az; };
	struct Frame {
		double stamp;
		int    width;
		int    height;
		std::vector<uint8_t> rgb;
		std::vector<uint8_t> depth;
	};

	// ── TCP server ───────────────────────────────────────────────
	bool startServer()
	{
#ifdef _WIN32
		WSADATA wsa;
		WSAStartup(MAKEWORD(2, 2), &wsa);
#endif
		listenSock_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (listenSock_ == INVALID_UNITY_SOCK) { UERROR("socket() failed"); return false; }

		int opt = 1;
		setsockopt(listenSock_, SOL_SOCKET, SO_REUSEADDR,
		           reinterpret_cast<const char *>(&opt), sizeof(opt));

		sockaddr_in addr{};
		addr.sin_family      = AF_INET;
		addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
		addr.sin_port        = htons(static_cast<uint16_t>(listenPort_));
		if (bind(listenSock_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0)
		{
			UERROR("bind() failed on port %d", listenPort_);
			closeServer();
			return false;
		}
		listen(listenSock_, 1);
		UINFO("CameraUnityTCP: listening on 127.0.0.1:%d", listenPort_);
		return true;
	}

	void closeServer()
	{
		if (listenSock_ != INVALID_UNITY_SOCK)
		{
			CLOSE_UNITY_SOCK(listenSock_);
			listenSock_ = INVALID_UNITY_SOCK;
		}
#ifdef _WIN32
		WSACleanup();
#endif
	}

	void closeClient()
	{
		if (clientSock_ != INVALID_UNITY_SOCK)
		{
			CLOSE_UNITY_SOCK(clientSock_);
			clientSock_ = INVALID_UNITY_SOCK;
		}
	}

	// ── Recv thread ──────────────────────────────────────────────
	void recvLoop()
	{
		while (running_)
		{
			// Accept (blocking — server socket is blocking by default)
			sockaddr_in caddr{};
			socklen_t   clen = sizeof(caddr);
			socket_t_unity s = accept(listenSock_,
			                          reinterpret_cast<sockaddr *>(&caddr), &clen);
			if (s == INVALID_UNITY_SOCK)
			{
				if (!running_) break;
				continue;
			}
			clientSock_ = s;
			UINFO("CameraUnityTCP: client connected");

			// Reset per-connection state
			calibrated_ = false;

			// Packet loop
			while (running_ && readAndDispatch()) {}

			closeClient();
			UINFO("CameraUnityTCP: client disconnected");
			// Loop back and accept again
		}
	}

	// Returns false on disconnect / error
	bool readAndDispatch()
	{
		uint8_t  type;
		uint32_t payloadSize;
		if (!recvAll(&type, 1))             return false;
		if (!recvAll(&payloadSize, 4))      return false;

		switch (type)
		{
			case 1: return handleCalib(payloadSize);
			case 2: return handleIMU(payloadSize);
			case 3: return handleRGBD(payloadSize);
			default:
				UWARN("CameraUnityTCP: unknown packet type %u (size %u) — skipping", type, payloadSize);
				return skipBytes(payloadSize);
		}
	}

	bool handleCalib(uint32_t payloadSize)
	{
		const uint32_t expected = 4 + 4 + 8 * 4 + 8 * 12;
		if (payloadSize != expected)
		{
			UERROR("Calib: expected %u bytes, got %u", expected, payloadSize);
			return false;
		}

		uint32_t w, h;
		double fx, fy, cx, cy;
		double lt[12];
		if (!recvAll(&w,  4)) return false;
		if (!recvAll(&h,  4)) return false;
		if (!recvAll(&fx, 8)) return false;
		if (!recvAll(&fy, 8)) return false;
		if (!recvAll(&cx, 8)) return false;
		if (!recvAll(&cy, 8)) return false;
		if (!recvAll(lt, sizeof(lt))) return false;

		Transform localTransform(
			(float)lt[0], (float)lt[1], (float)lt[2],  (float)lt[3],
			(float)lt[4], (float)lt[5], (float)lt[6],  (float)lt[7],
			(float)lt[8], (float)lt[9], (float)lt[10], (float)lt[11]);

		{
			std::lock_guard<std::mutex> lk(calibMtx_);
			width_  = static_cast<int>(w);
			height_ = static_cast<int>(h);
			model_  = CameraModel("unity_d455", fx, fy, cx, cy, localTransform, 0.0,
			                     cv::Size(width_, height_));
			// IMU local_transform: base_link → IMU body frame
			// Unity IMUSensor reports gyro/accel in its own body frame
			// (same axes as Unity camera: X-right, Y-up, Z-forward).
			// Conversion: base_link (X-fwd,Y-left,Z-up) → Unity body (X-right,Y-up,Z-fwd)
			baseToImu_ = Transform(0, -1, 0, 0,
			                       0,  0, 1, 0,
			                       1,  0, 0, 0);
			calibrated_ = true;
		}
		calibCv_.notify_all();
		return true;
	}

	bool handleIMU(uint32_t payloadSize)
	{
		if (payloadSize != 56) { UERROR("IMU: bad size %u", payloadSize); return false; }

		ImuSample s{};
		if (!recvAll(&s.stamp, 8)) return false;
		if (!recvAll(&s.gx,    8)) return false;
		if (!recvAll(&s.gy,    8)) return false;
		if (!recvAll(&s.gz,    8)) return false;
		if (!recvAll(&s.ax,    8)) return false;
		if (!recvAll(&s.ay,    8)) return false;
		if (!recvAll(&s.az,    8)) return false;

		std::lock_guard<std::mutex> lk(imuMtx_);
		if (imuQueue_.size() >= MAX_IMU_QUEUE)
			imuQueue_.pop_front();
		imuQueue_.push_back(s);
		return true;
	}

	bool handleRGBD(uint32_t payloadSize)
	{
		double   stamp;
		uint32_t w, h;
		if (!recvAll(&stamp, 8)) return false;
		if (!recvAll(&w,     4)) return false;
		if (!recvAll(&h,     4)) return false;

		size_t rgbSize   = static_cast<size_t>(w) * h * 3;
		size_t depthSize = static_cast<size_t>(w) * h * 2;
		uint32_t expected = 8 + 4 + 4 + static_cast<uint32_t>(rgbSize + depthSize);
		if (payloadSize != expected)
		{
			UERROR("RGBD: expected payload %u, got %u (w=%u h=%u)",
			       expected, payloadSize, w, h);
			return false;
		}

		Frame f;
		f.stamp  = stamp;
		f.width  = static_cast<int>(w);
		f.height = static_cast<int>(h);
		f.rgb.resize(rgbSize);
		f.depth.resize(depthSize);
		if (!recvAll(f.rgb.data(),   rgbSize))   return false;
		if (!recvAll(f.depth.data(), depthSize)) return false;

		{
			std::lock_guard<std::mutex> lk(frameMtx_);
			while (frameQueue_.size() >= MAX_FRAME_QUEUE)
				frameQueue_.pop_front();   // drop oldest
			frameQueue_.push_back(std::move(f));
		}
		frameCv_.notify_one();
		return true;
	}

	bool recvAll(void * buf, size_t n)
	{
		uint8_t * p = static_cast<uint8_t *>(buf);
		size_t left = n;
		while (left > 0)
		{
			int r = recv(clientSock_, reinterpret_cast<char *>(p), static_cast<int>(left), 0);
			if (r <= 0) return false;
			p    += r;
			left -= r;
		}
		return true;
	}

	bool skipBytes(uint32_t n)
	{
		uint8_t buf[4096];
		while (n > 0)
		{
			uint32_t chunk = n < sizeof(buf) ? n : (uint32_t)sizeof(buf);
			if (!recvAll(buf, chunk)) return false;
			n -= chunk;
		}
		return true;
	}

	// ── members ─────────────────────────────────────────────────
	int                       listenPort_;
	socket_t_unity            listenSock_;
	socket_t_unity            clientSock_;
	std::atomic<bool>         running_;

	std::thread               recvThread_;

	// Calibration state (set once after connect)
	mutable std::mutex        calibMtx_;
	std::condition_variable   calibCv_;
	std::atomic<bool>         calibrated_;
	int                       width_, height_;
	CameraModel               model_;
	Transform                 baseToImu_;

	// Frame queue (RGBD)
	std::mutex                frameMtx_;
	std::condition_variable   frameCv_;
	std::deque<Frame>         frameQueue_;
	static constexpr size_t   MAX_FRAME_QUEUE = 4;

	// IMU queue
	std::mutex                imuMtx_;
	std::deque<ImuSample>     imuQueue_;
	static constexpr size_t   MAX_IMU_QUEUE = 4096;
};

} // namespace rtabmap

#endif /* CAMERAUNITYTCP_H_ */
