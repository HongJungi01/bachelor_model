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
    pip install -r requirements.txt --index-url https://download.pytorch.org/whl/cu128
    if errorlevel 1 goto :err
) else (
    call .venv\Scripts\activate.bat
)

if exist .env.bat (
    call .env.bat
) else (
    echo [run_semantic] WARNING: .env.bat not found ^(copy .env.bat.example and add your API key^)
)

.venv\Scripts\python.exe -m uvicorn app:app --host 127.0.0.1 --port 7788
goto :eof

:err
echo [run_semantic] setup failed
exit /b 1
