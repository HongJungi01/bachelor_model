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

#ifndef RTABMAP_CAMERAUNITYTCP_H
#define RTABMAP_CAMERAUNITYTCP_H

#include <cstdint>

#ifdef _WIN32
typedef uintptr_t socket_t_unity;
#define INVALID_UNITY_SOCK ((uintptr_t)-1)
#else
typedef int socket_t_unity;
#define INVALID_UNITY_SOCK (-1)
#endif

#include <atomic>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>
#include <rtabmap/core/Camera.h>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/IMU.h>
#include <rtabmap/utilite/ULogger.h>

namespace rtabmap {

class RTABMAP_CORE_EXPORT CameraUnityTCP : public Camera
{
public:
	static bool available();

	CameraUnityTCP(int listenPort = 7778, float imageRate = 0.0f,
	               const Transform & localTransform = Transform::getIdentity());
	virtual ~CameraUnityTCP();

	virtual bool init(const std::string & calibrationFolder = ".",
	                  const std::string & cameraName = "") override;
	virtual bool isCalibrated() const override;
	virtual std::string getSerial() const override;

	void stop();

protected:
	virtual SensorData captureImage(SensorCaptureInfo * info = 0) override;

private:
	struct ImuSample { double stamp, gx, gy, gz, ax, ay, az; };
	struct Frame {
		double stamp;
		int width;
		int height;
		std::vector<uint8_t> rgb;
		std::vector<uint8_t> depth;
	};

	bool startServer();
	void closeServer();
	void closeClient();
	void recvLoop();
	bool readAndDispatch();
	bool handleCalib(uint32_t payloadSize);
	bool handleIMU(uint32_t payloadSize);
	bool handleRGBD(uint32_t payloadSize);
	bool recvAll(void * buf, size_t n);
	bool skipBytes(uint32_t n);

	int listenPort_;
	socket_t_unity listenSock_;
	socket_t_unity clientSock_;
	std::atomic<bool> running_;
	Transform localTransform_;

	std::thread recvThread_;

	mutable std::mutex calibMtx_;
	std::condition_variable calibCv_;
	std::atomic<bool> calibrated_;
	int width_;
	int height_;
	CameraModel model_;
	Transform baseToImu_;

	std::mutex frameMtx_;
	std::condition_variable frameCv_;
	std::deque<Frame> frameQueue_;
	static constexpr size_t MAX_FRAME_QUEUE = 4;

	std::mutex imuMtx_;
	std::deque<ImuSample> imuQueue_;
	static constexpr size_t MAX_IMU_QUEUE = 4096;
};

} // namespace rtabmap

#endif // RTABMAP_CAMERAUNITYTCP_H
