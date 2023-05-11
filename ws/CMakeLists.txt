# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ws)

FILE(GLOB app_sources src/*.c)
include_directories(../oslib/ws/)
target_sources(app PRIVATE src/main.c
                           ../oslib/ws/ws_sensors.c)