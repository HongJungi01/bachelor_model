@echo off
REM ============================================================
REM  run_pipeline.bat - rtabmap_pipeline.exe 실행 (Semantic SLAM)
REM
REM  사용법:
REM    run_pipeline.bat              → Unity + Mock 모드 (VLM 없이)
REM    run_pipeline.bat http         → Unity + GroundingDINO HTTP 모드
REM    run_pipeline.bat realsense    → RealSense + Mock 모드
REM    run_pipeline.bat realsense http → RealSense + HTTP 모드
REM
REM  HTTP 모드 사용 시 먼저 실행:
REM    semantic_service\run_semantic.bat
REM ============================================================

set PIPELINE=C:\dev\rtabmap_pipeline\build\Release\rtabmap_pipeline.exe
set RTABMAP_BIN=C:\dev\rtabmap\build\bin
set VCPKG_BIN=C:\dev\vcpkg_export\installed\x64-windows-release\bin

set PATH=%VCPKG_BIN%;%RTABMAP_BIN%;%PATH%

if not exist "%PIPELINE%" (
    echo ERROR: rtabmap_pipeline.exe not found.
    echo Build first: cmake --build c:\dev\rtabmap_pipeline\build --config Release
    pause
    exit /b 1
)

REM -- 인자 파싱 --
set SOURCE=unity
set SEMANTIC=mock

for %%A in (%*) do (
    if /i "%%A"=="realsense" set SOURCE=realsense
    if /i "%%A"=="http"      set SEMANTIC=http
)

if "%SEMANTIC%"=="http" (
    set SEM_ARGS=--semantic-url http://127.0.0.1:7788/detect
) else (
    set SEM_ARGS=--semantic mock
)

echo ============================================================
echo  Source  : %SOURCE%
echo  Semantic: %SEMANTIC%
echo  Output  : C:\dev\output\
echo ============================================================
echo.

"%PIPELINE%" --source %SOURCE% %SEM_ARGS% --output C:\dev\output
pause
