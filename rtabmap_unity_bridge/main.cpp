/*
 * main.cpp — Unity Virtual D455 Dataset → RTAB-Map SLAM → 2D Occupancy Grid
 *
 * Offline batch processing of RGBD datasets captured by Unity's RGBD_DataCollector.
 * Reads rgb_sync/ + depth_sync/ + imu.csv + d455_virtual.yaml from disk.
 * Outputs 2D occupancy grid via OpenCV window + TCP (7777) + PGM files.
 *
 * Based on rtabmap tools/CidSimsDataset/main.cpp (inline IMU parsing pattern).
 *
 * Usage:
 *   rtabmap_unity_offline.exe <dataset_path> [options]
 *
 * Options:
 *   --output <dir>       Output directory for PGM/YAML (default: <dataset>/output)
 *   --db <path>          Database path (default: <dataset>/rtabmap.db)
 *   --no-imu             Skip IMU data even if imu.csv exists
 *   --tcp-port <port>    TCP port for Unity streaming (default: 7777)
 *   --no-viz             Disable OpenCV visualization window
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
#include <rtabmap/core/Rtabmap.h>
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

// ── Global flag for Ctrl+C ──────────────────────────────────
static volatile std::sig_atomic_t g_running = 1;
static void signalHandler(int) { g_running = 0; }

using namespace rtabmap;

// ═══════════════════════════════════════════════════════════════
//  IMU CSV parser
// ═══════════════════════════════════════════════════════════════

// Parse one line: timestamp,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z
static bool parseIMULine(const std::string & line,
                         double & stamp,
                         cv::Vec3d & gyro,
                         cv::Vec3d & accel)
{
	if (line.empty() || line[0] == '#' || line[0] == 't') // skip header/comments
		return false;

	std::istringstream ss(line);
	char sep;
	if (!(ss >> stamp))   return false;
	if (!(ss >> sep))     return false;  // comma after timestamp
	if (!(ss >> gyro[0])) return false;
	if (!(ss >> sep))     return false;
	if (!(ss >> gyro[1])) return false;
	if (!(ss >> sep))     return false;
	if (!(ss >> gyro[2])) return false;
	if (!(ss >> sep))     return false;
	if (!(ss >> accel[0]))return false;
	if (!(ss >> sep))     return false;
	if (!(ss >> accel[1]))return false;
	if (!(ss >> sep))     return false;
	if (!(ss >> accel[2]))return false;
	return true;
}

// ═══════════════════════════════════════════════════════════════
//  Usage
// ═══════════════════════════════════════════════════════════════
static void printUsage()
{
	printf(
		"Usage: rtabmap_unity_offline <dataset_path> [options]\n"
		"\n"
		"  dataset_path        Folder containing rgb_sync/, depth_sync/,\n"
		"                      d455_virtual.yaml, and optionally imu.csv\n"
		"\n"
		"Options:\n"
		"  --output <dir>      Output directory (default: <dataset>/output)\n"
		"  --db <path>         Database file path (default: <dataset>/rtabmap.db)\n"
		"  --no-imu            Skip IMU data even if imu.csv exists\n"
		"  --tcp-port <port>   TCP port for grid streaming (default: 7777)\n"
		"  --no-viz            Disable OpenCV visualization window\n"
		"\n"
	);
}

// ═══════════════════════════════════════════════════════════════
//  Main
// ═══════════════════════════════════════════════════════════════
int main(int argc, char * argv[])
{
	// ── Signal handling ──
	std::signal(SIGINT,  signalHandler);
	std::signal(SIGTERM, signalHandler);

	// ── Logging ──
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kInfo);

	// ── Parse CLI arguments ──
	if (argc < 2)
	{
		printUsage();
		return 1;
	}

	std::string datasetPath = argv[1];
	std::string outputDir   = datasetPath + "/output";
	std::string dbPath      = datasetPath + "/rtabmap.db";
	int         tcpPort     = 7777;
	bool        useIMU      = true;
	bool        showViz     = true;

	for (int i = 2; i < argc; ++i)
	{
		if (strcmp(argv[i], "--output") == 0 && i + 1 < argc)
			outputDir = argv[++i];
		else if (strcmp(argv[i], "--db") == 0 && i + 1 < argc)
			dbPath = argv[++i];
		else if (strcmp(argv[i], "--tcp-port") == 0 && i + 1 < argc)
			tcpPort = std::atoi(argv[++i]);
		else if (strcmp(argv[i], "--no-imu") == 0)
			useIMU = false;
		else if (strcmp(argv[i], "--no-viz") == 0)
			showViz = false;
		else
		{
			UWARN("Unknown option: %s", argv[i]);
		}
	}

	// ── Validate dataset paths ──
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

	// ==============================================================
	//  1. Camera — CameraRGBDImages (offline disk reader)
	// ==============================================================
	//   depthScaleFactor = 1.0 (Unity depth is already in mm, CV_16UC1)
	//   imageRate = 0.0 (process as fast as possible)
	CameraRGBDImages * camera = new CameraRGBDImages(
		rgbPath,        // path to rgb_sync/
		depthPath,      // path to depth_sync/
		1.0f,           // depthScaleFactor: 1.0 = depth in mm as-is
		0.0f            // imageRate: 0 = max speed (offline)
	);

	// Filenames ARE timestamps (e.g. "1.234567.png" → stamp=1.234567)
	// No external timestamp file, no real-time sync (offline batch)
	camera->setTimestamps(true, "", false);

	// ==============================================================
	//  2. Parameters — D455 virtual camera optimised
	// ==============================================================
	ParametersMap params;

	// ── SLAM core ──
	params.insert(ParametersPair(Parameters::kRGBDCreateOccupancyGrid(), "true"));
	params.insert(ParametersPair(Parameters::kRtabmapPublishStats(),     "true"));

	// ── IMU / Gravity ──
	if (hasIMU)
	{
		params.insert(ParametersPair(Parameters::kOptimizerGravitySigma(), "0.3"));
		params.insert(ParametersPair(Parameters::kOdomAlignWithGround(),   "true"));
	}
	else
	{
		// No IMU — disable gravity-dependent features
		params.insert(ParametersPair(Parameters::kOptimizerGravitySigma(), "0"));
		params.insert(ParametersPair(Parameters::kOdomAlignWithGround(),   "false"));
	}

	// ── Grid parameters (same as rtabmap_2d_pipeline) ──
	params.insert(ParametersPair(Parameters::kGridCellSize(),            "0.05"));  // 5 cm
	params.insert(ParametersPair(Parameters::kGridRangeMax(),            "6.0"));   // D455 effective max
	params.insert(ParametersPair(Parameters::kGridRangeMin(),            "0.3"));   // D455 practical min
	params.insert(ParametersPair(Parameters::kGridDepthDecimation(),     "2"));     // 1280→640
	params.insert(ParametersPair(Parameters::kGridMaxObstacleHeight(),   "1.5"));
	params.insert(ParametersPair(Parameters::kGridMaxGroundHeight(),     "0.15"));
	params.insert(ParametersPair(Parameters::kGridNormalsSegmentation(), "true"));
	params.insert(ParametersPair(Parameters::kGridMaxGroundAngle(),      "45"));
	params.insert(ParametersPair(Parameters::kGridRayTracing(),          "true"));

	// ── Keyframe thresholds ──
	params.insert(ParametersPair(Parameters::kRGBDLinearUpdate(),  "0.05"));   // 5 cm
	params.insert(ParametersPair(Parameters::kRGBDAngularUpdate(), "0.087"));  // ~5 deg

	// ==============================================================
	//  3. SensorCaptureThread (used for postUpdate only, NOT threaded)
	// ==============================================================
	SensorCaptureThread cameraThread(camera, params);  // takes camera ownership

	if (hasIMU)
	{
		// Enable Complementary filter for IMU orientation estimation
		cameraThread.enableIMUFiltering(1, params);  // 1 = Complementary
	}

	// Init camera with calibration (loads d455_virtual.yaml)
	if (!cameraThread.camera()->init(datasetPath, "d455_virtual"))
	{
		UERROR("Camera init failed! Check calibration file: %s", calibFile.c_str());
		return 1;
	}

	UINFO("Camera initialised: %d images",
	      static_cast<CameraRGBDImages*>(cameraThread.camera())->imagesCount());

	// ==============================================================
	//  4. IMU file
	// ==============================================================
	std::ifstream imuStream;
	std::string imuLine;
	double nextImuStamp = 0.0;
	cv::Vec3d nextImuGyro, nextImuAccel;
	bool imuLineReady = false;

	// IMU localTransform: base_link → IMU frame
	// Unity camera frame: X-right, Y-up, Z-forward
	// RTAB-Map base_link: X-forward, Y-left, Z-up
	// baseToImu: base→IMU rotation
	//   IMU_x = -Base_y  (right = -left)
	//   IMU_y =  Base_z  (up = up)
	//   IMU_z =  Base_x  (forward = forward)
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
			// Skip header line
			std::getline(imuStream, imuLine);
			UINFO("IMU file opened: %s", imuFile.c_str());

			// Pre-read first IMU sample
			if (std::getline(imuStream, imuLine))
			{
				if (parseIMULine(imuLine, nextImuStamp, nextImuGyro, nextImuAccel))
					imuLineReady = true;
			}
		}
	}

	// ==============================================================
	//  5. Odometry
	// ==============================================================
	Odometry * odom = Odometry::create(params);

	// ==============================================================
	//  6. RTAB-Map
	// ==============================================================
	Rtabmap rtabmap;
	rtabmap.init(params, dbPath);

	// ==============================================================
	//  7. GridPublisher (offline direct-call mode)
	// ==============================================================
	GridPublisher gridPublisher(
		tcpPort,   // TCP port
		0.0,       // no rate limit in offline mode (process every keyframe)
		outputDir  // file-output directory
	);
	// NOT registered with UEventsManager — we call processOffline() directly

	// ==============================================================
	//  8. Processing loop
	// ==============================================================
	printf("=== RTAB-Map Unity Offline Pipeline ===\n");
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

		// ── 8a. Inject all IMU samples up to current image timestamp ──
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

				// Read next IMU line
				imuLineReady = false;
				if (std::getline(imuStream, imuLine))
				{
					if (parseIMULine(imuLine, nextImuStamp, nextImuGyro, nextImuAccel))
						imuLineReady = true;
				}
			}
		}

		// ── 8b. Process image frame ──
		cameraThread.postUpdate(&data, &cameraInfo);

		OdometryInfo odomInfo;
		Transform odomPose = odom->process(data, &odomInfo);

		totalFrames++;

		if (odomPose.isNull())
		{
			odomLost++;
			UWARN("Frame %d: odometry lost!", totalFrames);
		}
		else
		{
			// ── 8c. SLAM processing ──
			if (rtabmap.process(data, odomPose, odomInfo.reg.covariance))
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
		}

		// ── 8d. Next frame ──
		data = cameraThread.camera()->takeData(&cameraInfo);
	}

	// ==============================================================
	//  9. Finish
	// ==============================================================
	double elapsed = totalTimer.elapsed();

	printf("\n=== Processing Complete ===\n");
	printf("  Total frames:   %d\n", totalFrames);
	printf("  Keyframes:      %d\n", keyframes);
	printf("  Odometry lost:  %d\n", odomLost);
	printf("  Time:           %.1f s\n", elapsed);
	printf("  Avg speed:      %.1f fps\n", totalFrames / elapsed);

	// Save final map
	gridPublisher.saveFinalMap();
	printf("  Final map saved to: %s/map.pgm\n", outputDir.c_str());

	// Close database
	rtabmap.close(true);  // save all data
	printf("  Database saved: %s\n", dbPath.c_str());

	// Cleanup
	delete odom;
	if (imuStream.is_open())
		imuStream.close();

	printf("Done.\n");
	return 0;
}
