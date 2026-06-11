# RS485 full-duplex link test app — standalone ESP-IDF project, kept OUTSIDE
# firmware/ so it doesn't get pulled into the imager build's component scan.
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(rs485_bridge)
