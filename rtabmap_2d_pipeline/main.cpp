/*
 * main.cpp — RealSense D455f → RTAB-Map SLAM → 2D Occupancy Grid pipeline
 *
 * Headless (no Qt GUI).  Uses GridPublisher for:
 *   - OpenCV 2D map visualisation
 *   - TCP streaming to Unity (127.0.0.1:7777)
 *   - PGM+YAML file output
 *
 * Based on examples/RGBDMapping/main.cpp (RTAB-Map).
 *
 * Usage:
 *   rtabmap_2d_pipeline.exe [database_path]
 */

#include <csignal>
#include <cstdio>

#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/RtabmapThread.h>
#include <rtabmap/core/CameraRGBD.h>
#include <rtabmap/core/OdometryThread.h>
#include <rtabmap/core/SensorCaptureThread.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/utilite/ULogger.h>

#include "GridPublisher.h"

// ── Global flag for Ctrl+C ──────────────────────────────────
static volatile std::sig_atomic_t g_running = 1;
static void signalHandler(int) { g_running = 0; }

using namespace rtabmap;

int main(int argc, char * argv[])
{
	// ── Signal ──
	std::signal(SIGINT,  signalHandler);
	std::signal(SIGTERM, signalHandler);

	// ── Logging ──
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kInfo);

	// ── Database path (optional) ──
	std::string dbPath = "rtabmap.db";
	if (argc > 1) dbPath = argv[1];

	// ==============================================================
	//  1. Camera — RealSense D455f
	// ==============================================================
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

	// ==============================================================
	//  2. Parameters — D455f-optimised
	// ==============================================================
	ParametersMap params;

	// ── SLAM core ──
	params.insert(ParametersPair(Parameters::kRGBDCreateOccupancyGrid(), "true"));

	// ── D455f IMU / Gravity ──
	params.insert(ParametersPair(Parameters::kOptimizerGravitySigma(), "0.3"));
	params.insert(ParametersPair(Parameters::kOdomAlignWithGround(),   "true"));

	// ── Grid parameters ──
	params.insert(ParametersPair(Parameters::kGridCellSize(),            "0.05"));  // 5 cm
	params.insert(ParametersPair(Parameters::kGridRangeMax(),            "6.0"));   // D455f effective max
	params.insert(ParametersPair(Parameters::kGridRangeMin(),            "0.3"));   // D455f practical min
	params.insert(ParametersPair(Parameters::kGridDepthDecimation(),     "2"));     // 1280→640
	params.insert(ParametersPair(Parameters::kGridMaxObstacleHeight(),   "1.5"));   // furniture/people
	params.insert(ParametersPair(Parameters::kGridMaxGroundHeight(),     "0.15"));  // ground threshold
	params.insert(ParametersPair(Parameters::kGridNormalsSegmentation(), "true"));
	params.insert(ParametersPair(Parameters::kGridMaxGroundAngle(),      "45"));    // degrees
	params.insert(ParametersPair(Parameters::kGridRayTracing(),          "true"));  // fill free space

	// ── Keyframe thresholds (move at least 5 cm or 5° to add a new node) ──
	params.insert(ParametersPair(Parameters::kRGBDLinearUpdate(),   "0.05"));
	params.insert(ParametersPair(Parameters::kRGBDAngularUpdate(),  "0.087")); // ~5 deg

	// ==============================================================
	//  3. Odometry thread
	// ==============================================================
	OdometryThread odomThread(Odometry::create(params));

	// ==============================================================
	//  4. RTAB-Map thread
	// ==============================================================
	Rtabmap * rtabmap = new Rtabmap();
	rtabmap->init(params, dbPath);
	RtabmapThread rtabmapThread(rtabmap);  // takes ownership

	// ==============================================================
	//  5. GridPublisher — our custom event handler
	// ==============================================================
	GridPublisher gridPublisher(
		7777,    // TCP port
		0.1,     // min period = 10 Hz cap
		"output" // file-output directory
	);

	// ==============================================================
	//  6. Register handlers & create event pipe
	// ==============================================================
	odomThread.registerToEventsManager();
	rtabmapThread.registerToEventsManager();
	gridPublisher.registerToEventsManager();

	// Camera→Odometry pipe (RTAB-Map subscribes to OdometryEvent by default)
	UEventsManager::createPipe(&cameraThread, &odomThread, "SensorEvent");

	// ==============================================================
	//  7. Start threads
	// ==============================================================
	UINFO("Starting pipeline…");
	rtabmapThread.start();
	odomThread.start();
	cameraThread.start();

	printf("=== RTAB-Map 2D Pipeline running ===\n");
	printf("  Camera : RealSense D455f\n");
	printf("  TCP    : 127.0.0.1:7777\n");
	printf("  Output : output/map.pgm + map.yaml\n");
	printf("  Press Ctrl+C to stop.\n\n");

	// ==============================================================
	//  8. Main loop — wait for Ctrl+C
	// ==============================================================
	while (g_running)
	{
		uSleep(100);  // 100 ms idle
	}

	printf("\nShutting down…\n");

	// ==============================================================
	//  9. Cleanup
	// ==============================================================
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
