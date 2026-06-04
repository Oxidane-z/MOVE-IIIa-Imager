# Top-level CMakeLists for the CubeSat imaging firmware.
#
# Builds an ESP-IDF v5.4+ project targeting ESP32-P4 (M5Stamp-P4).
# See HOW_TO_BUILD.md for the workflow.

cmake_minimum_required(VERSION 3.16)

# The MI1602 thermal-camera driver is vendored in-tree at components/mi1602/
# (moved here June 2026 from the old sibling "MI1602 Dev/" folder so a fresh
# clone of this repo builds standalone). ESP-IDF auto-discovers components/,
# so no EXTRA_COMPONENT_DIRS is needed.

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(cubesat_imager)
