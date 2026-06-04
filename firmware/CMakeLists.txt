# Top-level CMakeLists for the CubeSat imaging firmware.
#
# Builds an ESP-IDF v5.4+ project targeting ESP32-P4 (M5Stamp-P4).
# See HOW_TO_BUILD.md for the workflow.

cmake_minimum_required(VERSION 3.16)

# Pull in the MI1602 driver from the sibling folder if it exists.
# The user develops it in `../../MI1602 Dev/components/mi1602/` so it can
# have its own git history; we just consume it via EXTRA_COMPONENT_DIRS.
# Note: spaces in path are fine; CMake handles them in a list context.
set(MI1602_COMPONENT_DIR "${CMAKE_CURRENT_LIST_DIR}/../../MI1602 Dev/components")
if(EXISTS "${MI1602_COMPONENT_DIR}/mi1602")
    list(APPEND EXTRA_COMPONENT_DIRS "${MI1602_COMPONENT_DIR}")
    message(STATUS "MI1602 driver found at ${MI1602_COMPONENT_DIR}/mi1602")
else()
    message(STATUS "MI1602 driver not present (expected at ${MI1602_COMPONENT_DIR}/mi1602) — build continues without it.")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(cubesat_imager)
