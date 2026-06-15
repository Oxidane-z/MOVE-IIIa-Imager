@echo off
:: Flash the RS485 test app to the board on the COM port passed as %1.
:: e.g.  cmd //c "...\_flash.bat" COM21
:: Same MSYSTEM= workaround as the firmware wrappers. Markers: === FLASH_OK ===
set "MSYSTEM="
set "MSYS2_ARG_CONV_EXCL="
call C:\esp\esp-idf-v6.0.1\export.bat
cd /d "C:\Users\zeyu.zhu\Pictures\SC850SL Dev\rs485_test"
echo === FLASH on %1 ===
idf.py -p %1 flash
if errorlevel 1 (
    echo FLASH_FAILED
    exit /b 1
)
echo === FLASH_OK ===
