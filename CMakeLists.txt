# Copyright (c) 2024 TDK Invensense
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_ZEPHYR_HAL_TDK_MODULE)

  zephyr_library()
  
  if(CONFIG_USE_EMD_ICM42670S OR CONFIG_USE_EMD_ICM42670P)

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
    if (NOT CONFIG_CPU_HAS_FPU STREQUAL "y")
      zephyr_compile_definitions(ICP101XX_DISABLE_FLOATING_POINT)
    endif()
    zephyr_library_sources(
      icp101xx/Icp101xx.c
    )
  endif()

  if(CONFIG_USE_EMD_ICT1531X)
    zephyr_include_directories(
      common
      ict1531x 
    )

    zephyr_library_sources(
      ict1531x/Ict1531x.c
    )
  endif()
endif()