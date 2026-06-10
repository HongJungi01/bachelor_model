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
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif

#include <rtabmap/core/camera/CameraUnityTCP.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/utilite/ULogger.h>

namespace rtabmap {

bool CameraUnityTCP::available()
{
	return true;
}

CameraUnityTCP::CameraUnityTCP(int listenPort, float imageRate,
                               const Transform & localTransform)
	: Camera(imageRate),
	  listenPort_(listenPort),
	  listenSock_(INVALID_UNITY_SOCK),
	  clientSock_(INVALID_UNITY_SOCK),
	  running_(false),
	  localTransform_(localTransform),
	  calibrated_(false),
	  width_(0),
	  height_(0)
{
}

CameraUnityTCP::~CameraUnityTCP()
{
	stop();
}

bool CameraUnityTCP::init(const std::string & /*calibrationFolder*/,
                          const std::string & /*cameraName*/)
{
	if (!startServer()) return false;
	running_ = true;
	recvThread_ = std::thread(&CameraUnityTCP::recvLoop, this);
	UINFO("CameraUnityTCP: listening on port %d (Unity may connect anytime)", listenPort_);
	return true;
}

bool CameraUnityTCP::isCalibrated() const
{
	// Always report calibrated: real intrinsics arrive via TCP after Unity connects.
	// captureImage() blocks on the first frame, by which time calibration has been
	// received (Unity sends Calib packet before any RGBD frame).
	return true;
}

std::string CameraUnityTCP::getSerial() const
{
	return "unity_tcp";
}

void CameraUnityTCP::stop()
{
	running_ = false;
	frameCv_.notify_all();
	calibCv_.notify_all();
	closeClient();
	closeServer();
	if (recvThread_.joinable()) recvThread_.join();
}

SensorData CameraUnityTCP::captureImage(SensorCaptureInfo * /*info*/)
{
	std::unique_lock<std::mutex> lk(frameMtx_);
	frameCv_.wait(lk, [this]{ return !frameQueue_.empty() || !running_; });
	if (!running_ || frameQueue_.empty()) return SensorData();

	Frame f = std::move(frameQueue_.front());
	frameQueue_.pop_front();
	lk.unlock();

	cv::Mat rgbView(f.height, f.width, CV_8UC3, f.rgb.data());
	cv::Mat rgb;
	cv::cvtColor(rgbView, rgb, cv::COLOR_RGB2BGR);

	cv::Mat depthView(f.height, f.width, CV_16UC1, f.depth.data());
	cv::Mat depth = depthView.clone();

	SensorData data(rgb, depth, model_, getNextSeqID(), f.stamp);
	return data;
}

bool CameraUnityTCP::startServer()
{
#ifdef _WIN32
	WSADATA wsa;
	WSAStartup(MAKEWORD(2, 2), &wsa);
	SOCKET s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	listenSock_ = (socket_t_unity)(uintptr_t)s;
#else
	int s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	listenSock_ = static_cast<socket_t_unity>(s);
#endif
	if (listenSock_ == INVALID_UNITY_SOCK) { UERROR("socket() failed"); return false; }

	int opt = 1;
#ifdef _WIN32
	setsockopt((SOCKET)(uintptr_t)(listenSock_), SOL_SOCKET, SO_REUSEADDR,
	           reinterpret_cast<const char *>(&opt), sizeof(opt));
#else
	setsockopt(static_cast<int>(listenSock_), SOL_SOCKET, SO_REUSEADDR,
	           reinterpret_cast<const char *>(&opt), sizeof(opt));
#endif

	sockaddr_in addr{};
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_ANY);  // Accept from any interface
	addr.sin_port = htons(static_cast<uint16_t>(listenPort_));
#ifdef _WIN32
	if (bind((SOCKET)(uintptr_t)(listenSock_), reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0)
#else
	if (bind(static_cast<int>(listenSock_), reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0)
#endif
	{
		UERROR("bind() failed on port %d", listenPort_);
		closeServer();
		return false;
	}
#ifdef _WIN32
	listen((SOCKET)(uintptr_t)(listenSock_), 1);
#else
	listen(static_cast<int>(listenSock_), 1);
#endif
	UINFO("CameraUnityTCP: listening on 0.0.0.0:%d", listenPort_);
	return true;
}

void CameraUnityTCP::closeServer()
{
	if (listenSock_ != INVALID_UNITY_SOCK)
	{
#ifdef _WIN32
		closesocket((SOCKET)(uintptr_t)(listenSock_));
#else
		close(static_cast<int>(listenSock_));
#endif
		listenSock_ = INVALID_UNITY_SOCK;
	}
#ifdef _WIN32
	WSACleanup();
#endif
}

void CameraUnityTCP::closeClient()
{
	if (clientSock_ != INVALID_UNITY_SOCK)
	{
#ifdef _WIN32
		closesocket((SOCKET)(uintptr_t)(clientSock_));
#else
		close(static_cast<int>(clientSock_));
#endif
		clientSock_ = INVALID_UNITY_SOCK;
	}
}

void CameraUnityTCP::recvLoop()
{
	while (running_)
	{
		sockaddr_in caddr{};
		socklen_t clen = sizeof(caddr);
#ifdef _WIN32
		SOCKET s = accept((SOCKET)(uintptr_t)(listenSock_),
		                   reinterpret_cast<sockaddr *>(&caddr), &clen);
		clientSock_ = (socket_t_unity)(uintptr_t)s;
#else
		int s = accept(static_cast<int>(listenSock_),
		               reinterpret_cast<sockaddr *>(&caddr), &clen);
		clientSock_ = static_cast<socket_t_unity>(s);
#endif
		if (clientSock_ == INVALID_UNITY_SOCK)
		{
			if (!running_) break;
			continue;
		}
		UINFO("CameraUnityTCP: client connected");

		calibrated_ = false;

		while (running_ && readAndDispatch()) {}

		closeClient();
		UINFO("CameraUnityTCP: client disconnected");
	}
}

bool CameraUnityTCP::readAndDispatch()
{
	uint8_t type;
	uint32_t payloadSize;
	if (!recvAll(&type, 1)) return false;
	if (!recvAll(&payloadSize, 4)) return false;

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

bool CameraUnityTCP::handleCalib(uint32_t payloadSize)
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
	if (!recvAll(&w, 4)) return false;
	if (!recvAll(&h, 4)) return false;
	if (!recvAll(&fx, 8)) return false;
	if (!recvAll(&fy, 8)) return false;
	if (!recvAll(&cx, 8)) return false;
	if (!recvAll(&cy, 8)) return false;
	if (!recvAll(lt, sizeof(lt))) return false;

	Transform localTransform(
		(float)lt[0], (float)lt[1], (float)lt[2], (float)lt[3],
		(float)lt[4], (float)lt[5], (float)lt[6], (float)lt[7],
		(float)lt[8], (float)lt[9], (float)lt[10], (float)lt[11]);

	// Unity is left-handed; if the sender folded the handedness flip into
	// this matrix it is a reflection (determinant -1) and every transform
	// chained on it corrupts the SLAM geometry. Detect it and fall back to
	// the standard optical rotation, keeping the translation.
	{
		const float det =
			localTransform.r11()*(localTransform.r22()*localTransform.r33() - localTransform.r23()*localTransform.r32())
			- localTransform.r12()*(localTransform.r21()*localTransform.r33() - localTransform.r23()*localTransform.r31())
			+ localTransform.r13()*(localTransform.r21()*localTransform.r32() - localTransform.r22()*localTransform.r31());
		if (det < 0.0f)
		{
			UWARN("CameraUnityTCP: camera local transform from Unity has "
			      "determinant %f (reflection). Replacing its rotation with "
			      "the standard optical rotation — fix the Unity sender.", det);
			const Transform optical = CameraModel::opticalRotation();
			localTransform = Transform(
				optical.r11(), optical.r12(), optical.r13(), localTransform.x(),
				optical.r21(), optical.r22(), optical.r23(), localTransform.y(),
				optical.r31(), optical.r32(), optical.r33(), localTransform.z());
		}
	}

	{
		std::lock_guard<std::mutex> lk(calibMtx_);
		width_ = static_cast<int>(w);
		height_ = static_cast<int>(h);
		model_ = CameraModel("unity_d455", fx, fy, cx, cy, localTransform, 0.0,
		                     cv::Size(width_, height_));
		// The previous value had determinant -1 — a reflection, not a
		// rotation (a left-handed Unity axis convention cannot be folded
		// into a rotation matrix; the handedness flip belongs on the
		// measurements themselves). Every Transform built from it spammed
		// "doesn't have normalized rotation" warnings. Odometry currently
		// ignores this IMU anyway (no orientation is sent), so use a proper
		// rotation; before ever enabling IMU filtering, verify the axis
		// mapping against the Unity sender (stationary car must read +9.81
		// along base z after this transform).
		baseToImu_ = Transform(0, 1, 0, 0,
		                       0, 0, 1, 0,
		                       1, 0, 0, 0);
		calibrated_ = true;
	}
	calibCv_.notify_all();
	return true;
}

bool CameraUnityTCP::handleIMU(uint32_t payloadSize)
{
	if (payloadSize != 56) { UERROR("IMU: bad size %u", payloadSize); return false; }

	ImuSample s{};
	if (!recvAll(&s.stamp, 8)) return false;
	if (!recvAll(&s.gx, 8)) return false;
	if (!recvAll(&s.gy, 8)) return false;
	if (!recvAll(&s.gz, 8)) return false;
	if (!recvAll(&s.ax, 8)) return false;
	if (!recvAll(&s.ay, 8)) return false;
	if (!recvAll(&s.az, 8)) return false;

	// Push IMU to the event-driven pipeline via Camera::postInterIMU()
	IMU imu(cv::Vec3d(s.gx, s.gy, s.gz),
	        cv::Mat::zeros(3, 3, CV_64FC1),
	        cv::Vec3d(s.ax, s.ay, s.az),
	        cv::Mat::zeros(3, 3, CV_64FC1),
	        baseToImu_);
	this->postInterIMU(imu, s.stamp);

	return true;
}

bool CameraUnityTCP::handleRGBD(uint32_t payloadSize)
{
	double stamp;
	uint32_t w, h;
	if (!recvAll(&stamp, 8)) return false;
	if (!recvAll(&w, 4)) return false;
	if (!recvAll(&h, 4)) return false;

	size_t rgbSize = static_cast<size_t>(w) * h * 3;
	size_t depthSize = static_cast<size_t>(w) * h * 2;
	uint32_t expected = 8 + 4 + 4 + static_cast<uint32_t>(rgbSize + depthSize);
	if (payloadSize != expected)
	{
		UERROR("RGBD: expected payload %u, got %u (w=%u h=%u)",
		       expected, payloadSize, w, h);
		return false;
	}

	Frame f;
	f.stamp = stamp;
	f.width = static_cast<int>(w);
	f.height = static_cast<int>(h);
	f.rgb.resize(rgbSize);
	f.depth.resize(depthSize);
	if (!recvAll(f.rgb.data(), rgbSize)) return false;
	if (!recvAll(f.depth.data(), depthSize)) return false;

	{
		std::lock_guard<std::mutex> lk(frameMtx_);
		while (frameQueue_.size() >= MAX_FRAME_QUEUE)
			frameQueue_.pop_front();
		frameQueue_.push_back(std::move(f));
	}
	frameCv_.notify_one();
	return true;
}

bool CameraUnityTCP::recvAll(void * buf, size_t n)
{
	uint8_t * p = static_cast<uint8_t *>(buf);
	size_t left = n;
	while (left > 0)
	{
#ifdef _WIN32
		int r = recv((SOCKET)(uintptr_t)(clientSock_), reinterpret_cast<char *>(p), static_cast<int>(left), 0);
#else
		ssize_t r = recv(static_cast<int>(clientSock_), reinterpret_cast<char *>(p), left, 0);
#endif
		if (r <= 0) return false;
		p += r;
		left -= r;
	}
	return true;
}

bool CameraUnityTCP::skipBytes(uint32_t n)
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

} // namespace rtabmap
