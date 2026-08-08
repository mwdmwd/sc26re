# SPDX-License-Identifier: AGPL-3.0-or-later

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(haptics_stereo)

target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../src)
target_sources(app PRIVATE
	src/main.c
	../../src/haptics.c
)
