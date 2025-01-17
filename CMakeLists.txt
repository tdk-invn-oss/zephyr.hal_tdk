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

  if(CONFIG_USE_EMD_ICP201XX)
    zephyr_include_directories(
      common/Invn
      icp201xx 
    )

    zephyr_library_sources(
      icp201xx/Icp201xx.c
      icp201xx/Icp201xxDriver.c
      icp201xx/Icp201xxSerif.c
    )
  endif()

  if(CONFIG_USE_EMD_ICP101XX)
    zephyr_include_directories(
      common/Invn 
      icp101xx 
    )

    # When building native_sim on Zephyr we need to explicitely disable messages
    zephyr_compile_definitions(-DINV_MSG_DISABLE=1)

    if (NOT CONFIG_CPU_HAS_FPU STREQUAL "y")
      zephyr_compile_definitions(ICP101XX_DISABLE_FLOATING_POINT)
    endif()
    zephyr_library_sources(
      icp101xx/Icp101xx.c
    )
  endif()

endif()
