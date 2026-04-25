@echo off
REM ═══════════════════════════════════════════════════════════
REM  run_unity.bat — Unity TCP streaming → real-time SLAM
REM
REM  Starts the pipeline as a TCP server on port 7778. Run Unity
REM  with RTABMapStreamer pointed at this host:port.
REM
REM  Usage:
REM    run_unity.bat [options]
REM
REM  Examples:
REM    run_unity.bat
REM    run_unity.bat --sensor-port 9000
REM    run_unity.bat --db C:\maps\session1.db --output C:\maps
REM ═══════════════════════════════════════════════════════════

set PATH=C:\dev\rtabmap\build\bin;C:\dev\vcpkg_export\installed\x64-windows-release\bin;%PATH%
set BUILD_DIR=%~dp0build\Release

if exist "%BUILD_DIR%\rtabmap_pipeline.exe" (
    "%BUILD_DIR%\rtabmap_pipeline.exe" --source unity %*
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
