@echo off
set "MSYSTEM="
set "MSYS2_ARG_CONV_EXCL="
call C:\esp\esp-idf-v6.0.1\export.bat > nul
cd /d "C:\Users\zeyu.zhu\Pictures\SC850SL Dev\firmware"
python -m esptool --chip esp32p4 -p COM16 --before usb-reset --after hard-reset chip-id > nul 2>&1
python capture_serial.py COM16 50
