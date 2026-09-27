# SPDX-License-Identifier: AGPL-3.0-or-later

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(imu_threshold)

set(IMU_EMUL_DIR ${ZEPHYR_BASE}/tests/drivers/sensor/lsm6dsv16x_stream/src)
target_include_directories(app PRIVATE ../../src ${IMU_EMUL_DIR})
target_sources(app PRIVATE
	src/main.c
	../../src/imu.c
	../../src/ibex_settings_registry.c
	${IMU_EMUL_DIR}/lsm6dsv16x_emul.c
)
