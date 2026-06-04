@echo off
:: Bash-via-MSYS leaks MSYSTEM into cmd, which makes export.bat bail.
set "MSYSTEM="
set "MSYS2_ARG_CONV_EXCL="
call C:\esp\esp-idf-v6.0.1\export.bat
cd /d "C:\Users\zeyu.zhu\Pictures\SC850SL Dev\firmware"
del /q sdkconfig 2>nul
echo === RECONFIGURE ===
idf.py reconfigure
if errorlevel 1 (
    echo BUILD_FAILED_AT_RECONFIGURE
    exit /b 1
)
echo === BUILD ===
idf.py build
if errorlevel 1 (
    echo BUILD_FAILED
    exit /b 1
)
echo === BUILD_OK ===
