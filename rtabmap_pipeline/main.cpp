/*
 * main.cpp — RTAB-Map → 2D Occupancy Grid pipeline (live + offline)
 *
 * Two modes:
 *   --source realsense                   Live RealSense D455f, threaded event pipeline
 *   --source images <dataset_path>       Offline Unity dataset (rgb_sync/, depth_sync/,
 *                                        d455_virtual.yaml, optional imu.csv), batch loop
 *
 * Both modes feed GridPublisher (OpenCV viz + TCP 7777 + PGM/YAML output).
 *
 * Usage:
 *   rtabmap_pipeline --source realsense [--db PATH] [--output DIR] [--tcp-port N]
 *   rtabmap_pipeline --source images <dataset_path> [--db PATH] [--output DIR]
 *                    [--tcp-port N] [--no-imu]
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
#include <fstream>
#include <sstream>
#include <string>

#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/OdometryThread.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/RtabmapThread.h>
#include <rtabmap/core/CameraRGBD.h>
#include <rtabmap/core/SensorCaptureThread.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/camera/CameraRGBDImages.h>
#include <rtabmap/core/IMU.h>
#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UDirectory.h>

#include "GridPublisher.h"

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

	// IMU / Gravity (D455 has IMU; Unity dataset may or may not)
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
	p.insert(ParametersPair(Parameters::kGridCellSize(),            "0.05"));  // 5 cm
	p.insert(ParametersPair(Parameters::kGridRangeMax(),            "6.0"));
	p.insert(ParametersPair(Parameters::kGridRangeMin(),            "0.3"));
	p.insert(ParametersPair(Parameters::kGridDepthDecimation(),     "2"));     // 1280→640
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
//  IMU CSV parser (offline mode)
//    timestamp,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z
// ═══════════════════════════════════════════════════════════════
static bool parseIMULine(const std::string & line,
                         double & stamp,
                         cv::Vec3d & gyro,
                         cv::Vec3d & accel)
{
	if (line.empty() || line[0] == '#' || line[0] == 't')
		return false;

	std::istringstream ss(line);
	char sep;
	if (!(ss >> stamp))    return false;
	if (!(ss >> sep))      return false;
	if (!(ss >> gyro[0]))  return false;
	if (!(ss >> sep))      return false;
	if (!(ss >> gyro[1]))  return false;
	if (!(ss >> sep))      return false;
	if (!(ss >> gyro[2]))  return false;
	if (!(ss >> sep))      return false;
	if (!(ss >> accel[0])) return false;
	if (!(ss >> sep))      return false;
	if (!(ss >> accel[1])) return false;
	if (!(ss >> sep))      return false;
	if (!(ss >> accel[2])) return false;
	return true;
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

	Camera * camera = new CameraRealSense2("", 0);  // auto-detect, max rate
	if (!camera->init())
	{
		UERROR("RealSense D455f init failed! Check USB 3.0 connection.");
		delete camera;
		return 1;
	}
	UINFO("Camera initialised.");

	SensorCaptureThread cameraThread(camera);  // takes ownership

	ParametersMap params = makeParams(true);

	OdometryThread odomThread(Odometry::create(params));

	Rtabmap * rtabmap = new Rtabmap();
	rtabmap->init(params, dbPath);
	RtabmapThread rtabmapThread(rtabmap);  // takes ownership

	GridPublisher gridPublisher(tcpPort, 0.1, outputDir);  // 10 Hz cap

	odomThread.registerToEventsManager();
	rtabmapThread.registerToEventsManager();
	gridPublisher.registerToEventsManager();

	UEventsManager::createPipe(&cameraThread, &odomThread, "SensorEvent");

	UINFO("Starting pipeline…");
	rtabmapThread.start();
	odomThread.start();
	cameraThread.start();

	printf("=== RTAB-Map Pipeline (live) ===\n");
	printf("  Camera : RealSense D455f\n");
	printf("  TCP    : 127.0.0.1:%d\n", tcpPort);
	printf("  Output : %s/map.pgm + map.yaml\n", outputDir.c_str());
	printf("  DB     : %s\n", dbPath.c_str());
	printf("  Press Ctrl+C to stop.\n\n");

	while (g_running)
		uSleep(100);

	printf("\nShutting down…\n");

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
//  Offline images path — batch loop with IMU interleaving
// ═══════════════════════════════════════════════════════════════
static int runOffline(const std::string & datasetPath,
                      const std::string & dbPath,
                      const std::string & outputDir,
                      int tcpPort,
                      bool useIMU)
{
	std::string rgbPath   = datasetPath + "/rgb_sync";
	std::string depthPath = datasetPath + "/depth_sync";
	std::string calibFile = datasetPath + "/d455_virtual.yaml";
	std::string imuFile   = datasetPath + "/imu.csv";

	if (!UDirectory::exists(rgbPath))
	{
		UERROR("RGB directory not found: %s", rgbPath.c_str());
		return 1;
	}
	if (!UDirectory::exists(depthPath))
	{
		UERROR("Depth directory not found: %s", depthPath.c_str());
		return 1;
	}
	if (!UFile::exists(calibFile))
	{
		UERROR("Calibration file not found: %s", calibFile.c_str());
		return 1;
	}

	bool hasIMU = useIMU && UFile::exists(imuFile);
	if (useIMU && !hasIMU)
		UWARN("IMU file not found: %s (proceeding without IMU)", imuFile.c_str());

	// Camera (offline disk reader)
	//   depthScaleFactor = 1.0 (Unity depth is already in mm, CV_16UC1)
	//   imageRate = 0.0 (process as fast as possible)
	CameraRGBDImages * camera = new CameraRGBDImages(rgbPath, depthPath, 1.0f, 0.0f);
	camera->setTimestamps(true, "", false);  // filenames are timestamps

	ParametersMap params = makeParams(hasIMU);

	SensorCaptureThread cameraThread(camera, params);  // takes camera ownership
	if (hasIMU)
		cameraThread.enableIMUFiltering(1, params);  // 1 = Complementary

	if (!cameraThread.camera()->init(datasetPath, "d455_virtual"))
	{
		UERROR("Camera init failed! Check calibration file: %s", calibFile.c_str());
		return 1;
	}

	UINFO("Camera initialised: %d images",
	      static_cast<CameraRGBDImages*>(cameraThread.camera())->imagesCount());

	// IMU stream
	std::ifstream imuStream;
	std::string   imuLine;
	double        nextImuStamp = 0.0;
	cv::Vec3d     nextImuGyro, nextImuAccel;
	bool          imuLineReady = false;

	// IMU localTransform: base_link → IMU frame
	// Unity camera frame: X-right, Y-up, Z-forward
	// RTAB-Map base_link: X-forward, Y-left, Z-up
	Transform baseToImu(0, -1, 0, 0,
	                    0,  0, 1, 0,
	                    1,  0, 0, 0);

	if (hasIMU)
	{
		imuStream.open(imuFile);
		if (!imuStream.is_open())
		{
			UWARN("Failed to open IMU file: %s", imuFile.c_str());
			hasIMU = false;
		}
		else
		{
			std::getline(imuStream, imuLine);  // skip header
			UINFO("IMU file opened: %s", imuFile.c_str());

			if (std::getline(imuStream, imuLine))
			{
				if (parseIMULine(imuLine, nextImuStamp, nextImuGyro, nextImuAccel))
					imuLineReady = true;
			}
		}
	}

	Odometry * odom = Odometry::create(params);

	Rtabmap rtabmap;
	rtabmap.init(params, dbPath);

	// Offline GridPublisher — direct call, no event registration, no rate limit
	GridPublisher gridPublisher(tcpPort, 0.0, outputDir);

	printf("=== RTAB-Map Pipeline (offline) ===\n");
	printf("  Dataset: %s\n", datasetPath.c_str());
	printf("  IMU:     %s\n", hasIMU ? "YES" : "NO");
	printf("  TCP:     127.0.0.1:%d\n", tcpPort);
	printf("  Output:  %s\n", outputDir.c_str());
	printf("  DB:      %s\n", dbPath.c_str());
	printf("  Press Ctrl+C to abort.\n\n");

	SensorCaptureInfo cameraInfo;
	SensorData data = cameraThread.camera()->takeData(&cameraInfo);

	int totalFrames = 0;
	int keyframes   = 0;
	int odomLost    = 0;
	UTimer totalTimer;

	while (data.isValid() && g_running)
	{
		double imageStamp = data.stamp();

		// Inject all IMU samples up to current image timestamp
		if (hasIMU)
		{
			while (imuLineReady && nextImuStamp <= imageStamp)
			{
				IMU imu(nextImuGyro,
				        cv::Mat::zeros(3, 3, CV_64FC1),
				        nextImuAccel,
				        cv::Mat::zeros(3, 3, CV_64FC1),
				        baseToImu);

				SensorData imuData(imu, 0, nextImuStamp);
				cameraThread.postUpdate(&imuData);
				odom->process(imuData);

				imuLineReady = false;
				if (std::getline(imuStream, imuLine))
				{
					if (parseIMULine(imuLine, nextImuStamp, nextImuGyro, nextImuAccel))
						imuLineReady = true;
				}
			}
		}

		// Process image frame
		cameraThread.postUpdate(&data, &cameraInfo);

		OdometryInfo odomInfo;
		Transform odomPose = odom->process(data, &odomInfo);

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
			{
				printf("  [%d/%d] Keyframes: %d, OdomLost: %d\n",
				       totalFrames,
				       (int)static_cast<CameraRGBDImages*>(cameraThread.camera())->imagesCount(),
				       keyframes, odomLost);
			}
		}

		data = cameraThread.camera()->takeData(&cameraInfo);
	}

	double elapsed = totalTimer.elapsed();

	printf("\n=== Processing Complete ===\n");
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

	delete odom;
	if (imuStream.is_open())
		imuStream.close();

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
		"  rtabmap_pipeline --source images <dataset_path> [options]\n"
		"\n"
		"Source modes:\n"
		"  --source realsense              Live RealSense D455f (threaded)\n"
		"  --source images <path>          Offline Unity dataset (batch)\n"
		"                                  Folder must contain rgb_sync/, depth_sync/,\n"
		"                                  d455_virtual.yaml, and optionally imu.csv\n"
		"\n"
		"Options:\n"
		"  --db <path>                     Database file path\n"
		"                                  (default: rtabmap.db | <dataset>/rtabmap.db)\n"
		"  --output <dir>                  PGM/YAML output directory\n"
		"                                  (default: output | <dataset>/output)\n"
		"  --tcp-port <port>               TCP port for grid streaming (default: 7777)\n"
		"  --no-imu                        Skip IMU data (offline mode only)\n"
		"\n"
	);
}

int main(int argc, char * argv[])
{
	std::signal(SIGINT,  signalHandler);
	std::signal(SIGTERM, signalHandler);

	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kInfo);

	enum Source { SRC_NONE, SRC_REALSENSE, SRC_IMAGES };
	Source      source      = SRC_NONE;
	std::string datasetPath;
	std::string dbPath;
	std::string outputDir;
	int         tcpPort     = 7777;
	bool        useIMU      = true;

	for (int i = 1; i < argc; ++i)
	{
		if (strcmp(argv[i], "--source") == 0 && i + 1 < argc)
		{
			std::string s = argv[++i];
			if (s == "realsense")
			{
				source = SRC_REALSENSE;
			}
			else if (s == "images")
			{
				source = SRC_IMAGES;
				if (i + 1 < argc && argv[i + 1][0] != '-')
					datasetPath = argv[++i];
			}
			else
			{
				UERROR("Unknown source: %s (expected 'realsense' or 'images')", s.c_str());
				printUsage();
				return 1;
			}
		}
		else if (strcmp(argv[i], "--db") == 0 && i + 1 < argc)
			dbPath = argv[++i];
		else if (strcmp(argv[i], "--output") == 0 && i + 1 < argc)
			outputDir = argv[++i];
		else if (strcmp(argv[i], "--tcp-port") == 0 && i + 1 < argc)
			tcpPort = std::atoi(argv[++i]);
		else if (strcmp(argv[i], "--no-imu") == 0)
			useIMU = false;
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

	if (source == SRC_NONE)
	{
		printUsage();
		return 1;
	}

	if (source == SRC_REALSENSE)
	{
		if (dbPath.empty())    dbPath    = "rtabmap.db";
		if (outputDir.empty()) outputDir = "output";
		return runLive(dbPath, outputDir, tcpPort);
	}

	// SRC_IMAGES
	if (datasetPath.empty())
	{
		UERROR("--source images requires a dataset path");
		printUsage();
		return 1;
	}
	if (dbPath.empty())    dbPath    = datasetPath + "/rtabmap.db";
	if (outputDir.empty()) outputDir = datasetPath + "/output";
	return runOffline(datasetPath, dbPath, outputDir, tcpPort, useIMU);
}
