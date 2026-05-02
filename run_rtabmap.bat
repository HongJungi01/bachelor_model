@echo off
REM ============================================================
REM  run_rtabmap.bat - Launch RTABMap GUI
REM
REM  Source 선택 (Preferences > Source):
REM    - RealSense2  : 실제 RealSense D455
REM    - Unity TCP   : Unity (unityCar)에서 TCP로 RGB+Depth+IMU 수신
REM                    (기본 Listen Port: 7778)
REM ============================================================

set RTABMAP_BIN=C:\dev\rtabmap\build\bin
set VCPKG_BIN=C:\dev\vcpkg_export\installed\x64-windows-release\bin
set QT_PLUGIN_PATH=C:\dev\vcpkg_export\installed\x64-windows-release\Qt6\plugins

REM Semantic SLAM: if the GroundingDINO sidecar (semantic_service\run_semantic.bat)
REM is running on 7788, the GUI auto-attaches and rasters per-keyframe masks
REM as obstacle (value 80) on the published grid. To disable, comment out the
REM next line or set it to empty.
set RTABMAP_SEMANTIC_URL=http://127.0.0.1:7788/detect

if not exist "%RTABMAP_BIN%\RTABMap.exe" (
    echo ERROR: RTABMap.exe not found at %RTABMAP_BIN%
    echo Build rtabmap first.
    pause
    exit /b 1
)

set PATH=%VCPKG_BIN%;%RTABMAP_BIN%;%PATH%
start "" "%RTABMAP_BIN%\RTABMap.exe"
