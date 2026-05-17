@echo off
setlocal
cd /d %~dp0

if not exist .venv (
    echo [run_semantic] creating venv ...
    python -m venv .venv
    if errorlevel 1 goto :err
    call .venv\Scripts\activate.bat
    echo [run_semantic] installing requirements ...
    pip install --upgrade pip
    pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128
    if errorlevel 1 goto :err
    pip install -r requirements.txt
    if errorlevel 1 goto :err
) else (
    call .venv\Scripts\activate.bat
)

if exist .env.bat (
    call .env.bat
)
if not exist models\best.pt (
    if "%YOLO_WEIGHTS%"=="" (
        echo [run_semantic] WARNING: models\best.pt not found and YOLO_WEIGHTS not set
        echo                place trained weights at models\best.pt or set YOLO_WEIGHTS=path\to\weights.pt
    )
)

.venv\Scripts\python.exe -m uvicorn app:app --host 127.0.0.1 --port 7788
goto :eof

:err
echo [run_semantic] setup failed
exit /b 1
