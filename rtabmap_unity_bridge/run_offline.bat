@echo off
REM ═══════════════════════════════════════════════════════════
REM  run_offline.bat — Unity Virtual D455 → RTAB-Map 오프라인 파이프라인
REM
REM  Usage:
REM    run_offline.bat <dataset_path> [options]
REM
REM  Example:
REM    run_offline.bat C:\D455_Dataset
REM    run_offline.bat C:\D455_Dataset --no-imu --tcp-port 8888
REM ═══════════════════════════════════════════════════════════

REM vcpkg 의존성 DLL + RTAB-Map DLL 경로
set PATH=C:\dev\rtabmap\build\bin;C:\dev\vcpkg_export\installed\x64-windows-release\bin;%PATH%

REM 빌드 출력 경로 (Release)
set BUILD_DIR=%~dp0build\Release

if "%~1"=="" (
    echo ═══════════════════════════════════════════════════════════
    echo  사용법: run_offline.bat ^<dataset_path^> [options]
    echo.
    echo  예시:
    echo    run_offline.bat C:\D455_Dataset
    echo    run_offline.bat C:\D455_Dataset --no-imu --tcp-port 8888
    echo ═══════════════════════════════════════════════════════════
    pause
    exit /b 1
)

if exist "%BUILD_DIR%\rtabmap_unity_offline.exe" (
    "%BUILD_DIR%\rtabmap_unity_offline.exe" %*
) else (
    echo ERROR: rtabmap_unity_offline.exe not found at %BUILD_DIR%
    echo.
    echo Build first:
    echo   cd %~dp0
    echo   mkdir build ^& cd build
    echo   cmake .. -DCMAKE_PREFIX_PATH="C:/dev/vcpkg_export/installed/x64-windows-release;C:/dev/rtabmap/build/install"
    echo   cmake --build . --config Release
)
pause
