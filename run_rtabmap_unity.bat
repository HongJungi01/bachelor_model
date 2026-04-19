@echo off
REM ═══════════════════════════════════════════════════════════
REM  run_rtabmap_unity.bat
REM
REM  Unity 가상 D455 RGBD 데이터셋을 RTABMap GUI에서 처리.
REM  기존 run_rtabmap.bat과 동일하게 GUI를 유지하며,
REM  소스만 RGBD Images + IMU CSV로 설정.
REM
REM  ■ 사전 설정 (최초 1회, GUI에서):
REM    Preferences → Source → RGB-D Camera → RGBD Images
REM      - RGB path:     C:/D455_Dataset/rgb_sync/
REM      - Depth path:   C:/D455_Dataset/depth_sync/
REM      - Depth scale:  1000  (16-bit mm → meter)
REM      - ☑ Filenames are timestamps
REM
REM    Preferences → Source → Images (옵션 패널):
REM      - IMU path:     C:/D455_Dataset/imu.csv
REM      - IMU local transform: 0 0 1 0 -1 0 0 0 0 -1 0 0
REM      - IMU rate:     0  (무제한)
REM
REM    Preferences → Source → Calibration:
REM      - calibration file: d455_virtual.yaml (데이터셋 폴더 내)
REM
REM  ■ 사용법:
REM    run_rtabmap_unity.bat
REM    → RTABMap.exe 실행 → Start 버튼 클릭
REM ═══════════════════════════════════════════════════════════

REM DLL 경로 설정
set PATH=C:\dev\rtabmap\build\bin;C:\dev\vcpkg_export\installed\x64-windows-release\bin;%PATH%

REM Qt 플러그인 경로
set QT_PLUGIN_PATH=C:\dev\vcpkg_export\installed\x64-windows-release\Qt6\plugins

echo =============================================
echo  RTABMap GUI - Unity Virtual D455 Dataset
echo =============================================
echo.
echo  [최초 설정 가이드]
echo  1. Preferences ^> Source ^> RGB-D Camera ^> RGBD Images
echo     - RGB path:    데이터셋/rgb_sync/
echo     - Depth path:  데이터셋/depth_sync/
echo     - Depth scale: 1000
echo     - Filenames are timestamps: 체크
echo.
echo  2. Preferences ^> Source ^> Images 옵션
echo     - IMU path:    데이터셋/imu.csv
echo     - IMU local transform: 0 0 1 0 -1 0 0 0 0 -1 0 0
echo.
echo  3. Start 버튼으로 SLAM 시작
echo =============================================
echo.

start "" "C:\dev\rtabmap\build\bin\RTABMap.exe"
