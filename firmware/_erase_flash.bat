@echo off
:: One-time full chip erase. Needed when the partition LAYOUT changes (e.g. the
:: single-factory -> dual-OTA switch), so the bootloader/otadata/nvs start from
:: a known-blank state instead of stale bytes from the previous layout. After
:: this, use _flash.bat / _flash_ground build normally.
::
:: Bash-via-MSYS leaks MSYSTEM into cmd, which makes export.bat bail.
set "MSYSTEM="
set "MSYS2_ARG_CONV_EXCL="
call C:\esp\esp-idf-v6.0.1\export.bat
cd /d "C:\Users\zeyu.zhu\Pictures\SC850SL Dev\firmware"
echo === ERASE_FLASH on COM16 ===
idf.py -p COM16 erase-flash
if errorlevel 1 (
    echo ERASE_FAILED
    exit /b 1
)
echo === ERASE_OK ===
