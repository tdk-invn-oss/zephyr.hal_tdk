# Copyright (c) 2024 TDK Invensense
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_TDK_HAL)

  zephyr_library()

  if(CONFIG_USE_EMD_ICM42670 OR CONFIG_USE_EMD_ICM42370)

    zephyr_include_directories(
      common
      icm42x7x
      icm42x7x/icm42x7x_h
    )

    zephyr_library_sources(
      icm42x7x/imu/inv_imu_driver.c
      icm42x7x/imu/inv_imu_transport.c
    )
    zephyr_library_sources_ifdef(CONFIG_TDK_APEX icm42x7x/imu/inv_imu_apex.c)

  endif()

endif()
