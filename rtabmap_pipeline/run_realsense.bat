@echo off
REM ═══════════════════════════════════════════════════════════
REM  run_realsense.bat — RealSense D455f live SLAM (threaded)
REM
REM  Usage:
REM    run_realsense.bat [extra args]
REM
REM  Examples:
REM    run_realsense.bat
REM    run_realsense.bat --db C:\maps\session1.db
REM    run_realsense.bat --output C:\maps --tcp-port 8888
REM ═══════════════════════════════════════════════════════════

set PATH=C:\dev\rtabmap\build\bin;C:\dev\vcpkg_export\installed\x64-windows-release\bin;%PATH%
set BUILD_DIR=%~dp0build\Release

if exist "%BUILD_DIR%\rtabmap_pipeline.exe" (
    "%BUILD_DIR%\rtabmap_pipeline.exe" --source realsense %*
) else (
    echo ERROR: rtabmap_pipeline.exe not found at %BUILD_DIR%
    echo.
    echo Build first:
    echo   cd %~dp0
    echo   mkdir build ^& cd build
    echo   cmake .. -DCMAKE_PREFIX_PATH="C:/dev/vcpkg_export/installed/x64-windows-release;C:/dev/rtabmap/build/install"
    echo   cmake --build . --config Release
    pause
)
