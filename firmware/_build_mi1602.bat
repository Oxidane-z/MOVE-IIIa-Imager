@echo off
:: MI1602 thermal-only debug build: base flight defaults + the MI1602_TEST
:: overlay that flips the camera target. The flight build (_build.bat) and Tab5
:: build are unaffected. Same === BUILD_OK === / BUILD_FAILED markers.
::
:: Bash-via-MSYS leaks MSYSTEM into cmd, which makes export.bat bail.
set "MSYSTEM="
set "MSYS2_ARG_CONV_EXCL="
call C:\esp\esp-idf-v6.0.1\export.bat
cd /d "C:\Users\zeyu.zhu\Pictures\SC850SL Dev\firmware"
set "SDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.defaults.mi1602"
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
