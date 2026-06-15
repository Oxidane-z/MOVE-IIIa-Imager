@echo off
:: Incremental build (no set-target) — for fast baud-sweep iterations.
set "MSYSTEM="
set "MSYS2_ARG_CONV_EXCL="
call C:\esp\esp-idf-v6.0.1\export.bat
cd /d "C:\Users\zeyu.zhu\Pictures\SC850SL Dev\rs485_test"
idf.py build
if errorlevel 1 (
    echo BUILD_FAILED
    exit /b 1
)
echo === BUILD_OK ===
