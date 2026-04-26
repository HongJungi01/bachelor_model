/*
 * main.cpp — RTAB-Map → 2D Occupancy Grid pipeline (live RealSense + Unity TCP)
 *
 * Two modes:
 *   --source realsense                   Live RealSense D455f, threaded event pipeline
 *   --source unity                       Unity client streams RGB+Depth+IMU over TCP
 *
 * Both modes feed GridPublisher (OpenCV viz + TCP 7777 + PGM/YAML output).
 *
 * Usage:
 *   rtabmap_pipeline --source realsense [--db PATH] [--output DIR] [--tcp-port N]
 *   rtabmap_pipeline --source unity     [--db PATH] [--output DIR] [--tcp-port N]
 *                                       [--sensor-port N]
 */

// Must come before ANY other includes to prevent winsock.h/winsock2.h conflict
#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#endif

#include <csignal>
#include <cstdio>
#include <cstring>
#include <string>

#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/OdometryThread.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/RtabmapThread.h>
#include <rtabmap/core/CameraRGBD.h>
#include <rtabmap/core/SensorCaptureThread.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/IMU.h>
#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UTimer.h>

#include "GridPublisher.h"
#include "CameraUnityTCP.h"

static volatile std::sig_atomic_t g_running = 1;
static void signalHandler(int) { g_running = 0; }

using namespace rtabmap;

// ═══════════════════════════════════════════════════════════════
//  Shared SLAM/grid parameters
// ═══════════════════════════════════════════════════════════════
static ParametersMap makeParams(bool hasIMU)
{
	ParametersMap p;

	// SLAM core
	p.insert(ParametersPair(Parameters::kRGBDCreateOccupancyGrid(), "true"));
	p.insert(ParametersPair(Parameters::kRtabmapPublishStats(),     "true"));

	// IMU / Gravity
	if (hasIMU)
	{
		p.insert(ParametersPair(Parameters::kOptimizerGravitySigma(), "0.3"));
		p.insert(ParametersPair(Parameters::kOdomAlignWithGround(),   "true"));
	}
	else
	{
		p.insert(ParametersPair(Parameters::kOptimizerGravitySigma(), "0"));
		p.insert(ParametersPair(Parameters::kOdomAlignWithGround(),   "false"));
	}

	// Grid parameters (D455 effective range)
	p.insert(ParametersPair(Parameters::kGridCellSize(),            "0.05"));
	p.insert(ParametersPair(Parameters::kGridRangeMax(),            "6.0"));
	p.insert(ParametersPair(Parameters::kGridRangeMin(),            "0.3"));
	p.insert(ParametersPair(Parameters::kGridDepthDecimation(),     "2"));
	p.insert(ParametersPair(Parameters::kGridMaxObstacleHeight(),   "1.5"));
	p.insert(ParametersPair(Parameters::kGridMaxGroundHeight(),     "0.15"));
	p.insert(ParametersPair(Parameters::kGridNormalsSegmentation(), "true"));
	p.insert(ParametersPair(Parameters::kGridMaxGroundAngle(),      "45"));
	p.insert(ParametersPair(Parameters::kGridRayTracing(),          "true"));

	// Keyframe thresholds: add a node every 5 cm or ~5°
	p.insert(ParametersPair(Parameters::kRGBDLinearUpdate(),  "0.05"));
	p.insert(ParametersPair(Parameters::kRGBDAngularUpdate(), "0.087"));

	return p;
}

// ═══════════════════════════════════════════════════════════════
//  Live RealSense path — threaded event pipeline
// ═══════════════════════════════════════════════════════════════
static int runLive(const std::string & dbPath,
                   const std::string & outputDir,
                   int tcpPort)
{
	if (!CameraRealSense2::available())
	{
		UERROR("Not built with RealSense2 SDK support. Rebuild with -DWITH_REALSENSE2=ON");
		return 1;
	}

	Camera * camera = new CameraRealSense2("", 0);
	if (!camera->init())
	{
		UERROR("RealSense D455f init failed! Check USB 3.0 connection.");
		delete camera;
		return 1;
	}
	UINFO("Camera initialised.");

	SensorCaptureThread cameraThread(camera);

	ParametersMap params = makeParams(true);

	OdometryThread odomThread(Odometry::create(params));

	Rtabmap * rtabmap = new Rtabmap();
	rtabmap->init(params, dbPath);
	RtabmapThread rtabmapThread(rtabmap);

	GridPublisher gridPublisher(tcpPort, 0.1, outputDir);

	odomThread.registerToEventsManager();
	rtabmapThread.registerToEventsManager();
	gridPublisher.registerToEventsManager();

	UEventsManager::createPipe(&cameraThread, &odomThread, "SensorEvent");

	UINFO("Starting pipeline...");
	rtabmapThread.start();
	odomThread.start();
	cameraThread.start();

	printf("=== RTAB-Map Pipeline (live RealSense) ===\n");
	printf("  Camera : RealSense D455f\n");
	printf("  TCP    : 127.0.0.1:%d (grid out)\n", tcpPort);
	printf("  Output : %s/map.pgm + map.yaml\n", outputDir.c_str());
	printf("  DB     : %s\n", dbPath.c_str());
	printf("  Press Ctrl+C to stop.\n\n");

	while (g_running)
		uSleep(100);

	printf("\nShutting down...\n");

	gridPublisher.setRunning(false);
	gridPublisher.unregisterFromEventsManager();
	rtabmapThread.unregisterFromEventsManager();
	odomThread.unregisterFromEventsManager();

	cameraThread.kill();
	odomThread.join(true);
	rtabmapThread.join(true);

	rtabmap->close(false);
	printf("Done.\n");
	return 0;
}

// ═══════════════════════════════════════════════════════════════
//  Unity TCP path — Unity streams RGB+Depth+IMU; we run the
//  same single-threaded SLAM loop that the offline mode used.
// ═══════════════════════════════════════════════════════════════
static int runUnity(const std::string & dbPath,
                    const std::string & outputDir,
                    int gridTcpPort,
                    int sensorTcpPort)
{
	CameraUnityTCP * camera = new CameraUnityTCP(sensorTcpPort, 0.0f);
	if (!camera->init())
	{
		UERROR("CameraUnityTCP init failed (no client connected within timeout).");
		delete camera;
		return 1;
	}

	ParametersMap params = makeParams(true);   // Unity always sends IMU

	SensorCaptureThread cameraThread(camera, params);   // takes ownership
	cameraThread.enableIMUFiltering(1, params);          // Complementary filter

	Odometry * odom = Odometry::create(params);

	Rtabmap rtabmap;
	rtabmap.init(params, dbPath);

	GridPublisher gridPublisher(gridTcpPort, 0.0, outputDir);

	printf("=== RTAB-Map Pipeline (Unity TCP) ===\n");
	printf("  Sensor: 127.0.0.1:%d (RGB+Depth+IMU in)\n", sensorTcpPort);
	printf("  Grid  : 127.0.0.1:%d (occupancy out)\n",    gridTcpPort);
	printf("  Output: %s\n", outputDir.c_str());
	printf("  DB    : %s\n", dbPath.c_str());
	printf("  Press Ctrl+C to stop.\n\n");

	int totalFrames = 0;
	int keyframes   = 0;
	int odomLost    = 0;
	UTimer totalTimer;

	auto * unityCam = static_cast<CameraUnityTCP *>(cameraThread.camera());

	SensorCaptureInfo cameraInfo;
	SensorData data = cameraThread.camera()->takeData(&cameraInfo);

	while (data.isValid() && g_running)
	{
		double imageStamp = data.stamp();

		// Drain IMU samples up to current image timestamp
		unityCam->drainIMU(imageStamp,
			[&](const IMU & imu, double stamp)
			{
				SensorData imuData(imu, 0, stamp);
				cameraThread.postUpdate(&imuData);
				odom->process(imuData);
			});

		cameraThread.postUpdate(&data, &cameraInfo);

		OdometryInfo odomInfo;
		Transform    odomPose = odom->process(data, &odomInfo);

		totalFrames++;

		if (odomPose.isNull())
		{
			odomLost++;
			UWARN("Frame %d: odometry lost!", totalFrames);
		}
		else if (rtabmap.process(data, odomPose, odomInfo.reg.covariance))
		{
			keyframes++;
			Statistics stats = rtabmap.getStatistics();
			gridPublisher.processOffline(stats, odomPose);

			if (keyframes % 10 == 0)
				printf("  Keyframes: %d, OdomLost: %d, Frames: %d\n",
				       keyframes, odomLost, totalFrames);
		}

		data = cameraThread.camera()->takeData(&cameraInfo);
	}

	double elapsed = totalTimer.elapsed();

	printf("\n=== Stopped ===\n");
	printf("  Total frames:   %d\n", totalFrames);
	printf("  Keyframes:      %d\n", keyframes);
	printf("  Odometry lost:  %d\n", odomLost);
	printf("  Time:           %.1f s\n", elapsed);
	if (elapsed > 0.0)
		printf("  Avg speed:      %.1f fps\n", totalFrames / elapsed);

	gridPublisher.saveFinalMap();
	printf("  Final map saved to: %s/map.pgm\n", outputDir.c_str());

	rtabmap.close(true);
	printf("  Database saved: %s\n", dbPath.c_str());

	unityCam->stop();
	delete odom;

	printf("Done.\n");
	return 0;
}

// ═══════════════════════════════════════════════════════════════
//  CLI
// ═══════════════════════════════════════════════════════════════
static void printUsage()
{
	printf(
		"Usage:\n"
		"  rtabmap_pipeline --source realsense [options]\n"
		"  rtabmap_pipeline --source unity     [options]\n"
		"\n"
		"Source modes:\n"
		"  --source realsense              Live RealSense D455f (threaded)\n"
		"  --source unity                  Unity client streams RGBD+IMU over TCP\n"
		"\n"
		"Options:\n"
		"  --db <path>                     Database file path (default: rtabmap.db)\n"
		"  --output <dir>                  PGM/YAML output directory (default: output)\n"
		"  --tcp-port <port>               Grid TCP port (default: 7777)\n"
		"  --sensor-port <port>            Unity sensor TCP port (default: 7778)\n"
		"\n"
	);
}

int main(int argc, char * argv[])
{
	std::signal(SIGINT,  signalHandler);
	std::signal(SIGTERM, signalHandler);

	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kInfo);

	enum Source { SRC_NONE, SRC_REALSENSE, SRC_UNITY };
	Source      source        = SRC_NONE;
	std::string dbPath;
	std::string outputDir;
	int         gridTcpPort   = 7777;
	int         sensorTcpPort = 7778;

	for (int i = 1; i < argc; ++i)
	{
		if (strcmp(argv[i], "--source") == 0 && i + 1 < argc)
		{
			std::string s = argv[++i];
			if      (s == "realsense") source = SRC_REALSENSE;
			else if (s == "unity")     source = SRC_UNITY;
			else
			{
				UERROR("Unknown source: %s (expected 'realsense' or 'unity')", s.c_str());
				printUsage();
				return 1;
			}
		}
		else if (strcmp(argv[i], "--db") == 0          && i + 1 < argc) dbPath        = argv[++i];
		else if (strcmp(argv[i], "--output") == 0      && i + 1 < argc) outputDir     = argv[++i];
		else if (strcmp(argv[i], "--tcp-port") == 0    && i + 1 < argc) gridTcpPort   = std::atoi(argv[++i]);
		else if (strcmp(argv[i], "--sensor-port") == 0 && i + 1 < argc) sensorTcpPort = std::atoi(argv[++i]);
		else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0)
		{
			printUsage();
			return 0;
		}
		else
		{
			UWARN("Unknown option: %s", argv[i]);
		}
	}

	if (source == SRC_NONE) { printUsage(); return 1; }

	if (dbPath.empty())    dbPath    = "rtabmap.db";
	if (outputDir.empty()) outputDir = "output";

	if (source == SRC_REALSENSE)
		return runLive(dbPath, outputDir, gridTcpPort);
	else
		return runUnity(dbPath, outputDir, gridTcpPort, sensorTcpPort);
}
