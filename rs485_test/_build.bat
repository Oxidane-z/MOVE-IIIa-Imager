@echo off
:: RS485 test-app build. Same MSYSTEM= workaround as firmware\_build.bat
:: (export.bat bails when MSYSTEM leaks in from MSYS bash). Markers:
:: === BUILD_OK === / BUILD_FAILED.
set "MSYSTEM="
set "MSYS2_ARG_CONV_EXCL="
call C:\esp\esp-idf-v6.0.1\export.bat
cd /d "C:\Users\zeyu.zhu\Pictures\SC850SL Dev\rs485_test"
echo === SET-TARGET ===
idf.py set-target esp32p4
if errorlevel 1 (
    echo BUILD_FAILED
    exit /b 1
)
echo === BUILD ===
idf.py build
if errorlevel 1 (
    echo BUILD_FAILED
    exit /b 1
)
echo === BUILD_OK ===
