# Copyright (c) 2024 TDK Invensense
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(
  common 
  icm42x7x 
)

if(CONFIG_USE_EMD_ICM42670P)
  zephyr_include_directories(icm42x7x/icm42670p_h)
endif()

if(CONFIG_USE_EMD_ICM42670S)
  zephyr_include_directories(icm42x7x/icm42670s_h)
endif()

zephyr_library()

zephyr_library_sources(
  icm42x7x/imu/inv_imu_driver.c 
  icm42x7x/imu/inv_imu_transport.c
)

zephyr_library_sources_ifdef(CONFIG_TDK_APEX icm42x7x/imu/inv_imu_apex.c)
