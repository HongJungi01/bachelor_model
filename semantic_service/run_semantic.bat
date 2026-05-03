@echo off
REM ============================================================
REM   semantic_service launcher
REM
REM   First run creates a venv and installs requirements.txt.
REM   For CUDA torch, install the cu121 wheel manually after
REM   the first run finishes:
REM
REM     .venv\Scripts\activate
REM     pip uninstall -y torch
REM     pip install torch --index-url https://download.pytorch.org/whl/cu121
REM
REM   Subsequent runs just activate and start uvicorn.
REM ============================================================

setlocal
if exist %~dp0.env.bat (
    call %~dp0.env.bat
) else (
    echo [semantic_service] WARNING: .env.bat not found - create it from .env.bat.example
)
cd /d %~dp0

if not exist .venv\Scripts\activate.bat (
    echo [semantic_service] creating venv ...
    py -3.11 -m venv .venv || (
        echo [semantic_service] python 3.11 not found - falling back to default python
        python -m venv .venv || exit /b 1
    )
    call .venv\Scripts\activate.bat
    python -m pip install --upgrade pip
    python -m pip install -r requirements.txt || exit /b 1
) else (
    call .venv\Scripts\activate.bat
)

python -m uvicorn app:app --host 127.0.0.1 --port 7788
