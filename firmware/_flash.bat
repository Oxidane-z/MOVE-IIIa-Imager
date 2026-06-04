@echo off
:: Bash-via-MSYS leaks MSYSTEM into cmd, which makes export.bat bail.
set "MSYSTEM="
set "MSYS2_ARG_CONV_EXCL="
call C:\esp\esp-idf-v6.0.1\export.bat
cd /d "C:\Users\zeyu.zhu\Pictures\SC850SL Dev\firmware"
echo === FLASH on COM16 ===
idf.py -p COM16 flash
if errorlevel 1 (
    echo FLASH_FAILED
    exit /b 1
)
echo === FLASH_OK ===
