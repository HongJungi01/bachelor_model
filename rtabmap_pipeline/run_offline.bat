@echo off
REM ═══════════════════════════════════════════════════════════
REM  run_offline.bat — Unity Virtual D455 dataset → offline batch SLAM
REM
REM  Usage:
REM    run_offline.bat <dataset_path> [options]
REM
REM  Examples:
REM    run_offline.bat C:\D455_Dataset
REM    run_offline.bat C:\D455_Dataset --no-imu
REM    run_offline.bat C:\D455_Dataset --tcp-port 8888 --output C:\maps
REM ═══════════════════════════════════════════════════════════

set PATH=C:\dev\rtabmap\build\bin;C:\dev\vcpkg_export\installed\x64-windows-release\bin;%PATH%
set BUILD_DIR=%~dp0build\Release

if "%~1"=="" (
    echo ═══════════════════════════════════════════════════════════
    echo  Usage: run_offline.bat ^<dataset_path^> [options]
    echo.
    echo  Example:
    echo    run_offline.bat C:\D455_Dataset
    echo    run_offline.bat C:\D455_Dataset --no-imu --tcp-port 8888
    echo ═══════════════════════════════════════════════════════════
    pause
    exit /b 1
)

if exist "%BUILD_DIR%\rtabmap_pipeline.exe" (
    "%BUILD_DIR%\rtabmap_pipeline.exe" --source images %*
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
