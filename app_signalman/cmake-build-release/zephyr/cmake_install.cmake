# Install script for directory: /home/rts/zephyr-rts-master/zephyr

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/opt/zephyr-sdk/arm-zephyr-eabi/bin/arm-zephyr-eabi-objdump")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/zephyr/arch/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/zephyr/lib/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/zephyr/soc/arm/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/zephyr/boards/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/zephyr/subsys/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/zephyr/drivers/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/modules/CANopenNode/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/modules/civetweb/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/modules/cmsis/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/modules/fatfs/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/modules/altera/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/modules/atmel/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/modules/cypress/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/modules/espressif/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/modules/hal_gigadevice/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/modules/infineon/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/modules/microchip/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/modules/hal_nordic/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/modules/nuvoton/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/modules/hal_nxp/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/modules/openisa/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/modules/quicklogic/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/modules/hal_rpi_pico/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/modules/silabs/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/modules/st/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/modules/stm32/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/modules/telink/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/modules/ti/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/modules/xtensa/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/modules/libmetal/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/modules/littlefs/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/modules/loramac-node/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/modules/lvgl/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/modules/lz4/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/modules/mbedtls/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/modules/mcuboot/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/modules/mipi-sys-t/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/modules/nanopb/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/modules/nrf_hw_models/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/modules/open-amp/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/modules/openthread/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/modules/segger/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/modules/sof/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/modules/tflite-micro/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/modules/tinycbor/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/modules/tinycrypt/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/modules/TraceRecorder/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/modules/trusted-firmware-m/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/modules/zscilib/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/modules/rts-zephyr/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/zephyr/kernel/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/zephyr/cmake/flash/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/zephyr/cmake/usage/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rts/tmp/projects/beacon_project_rzd/lora_russia_railways/app_signalman/cmake-build-release/zephyr/cmake/reports/cmake_install.cmake")
endif()

