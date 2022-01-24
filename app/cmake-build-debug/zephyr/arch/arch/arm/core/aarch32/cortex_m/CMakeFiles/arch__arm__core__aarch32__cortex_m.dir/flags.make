# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.21

# compile ASM with /opt/zephyr-sdk/arm-zephyr-eabi/bin/arm-zephyr-eabi-gcc
# compile C with /opt/zephyr-sdk/arm-zephyr-eabi/bin/arm-zephyr-eabi-gcc
ASM_DEFINES = -DBUILD_VERSION=zephyr-v2.7.0 -DCORE_CM4 -DHSE_VALUE=8000000 -DKERNEL -DSTM32L452xx -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -D_FORTIFY_SOURCE=2 -D__LINUX_ERRNO_EXTENSIONS__ -D__PROGRAM_START -D__ZEPHYR_SUPERVISOR__ -D__ZEPHYR__=1

ASM_INCLUDES = -I/home/rts/zephyr-2.7.0/zephyr/kernel/include -I/home/rts/zephyr-2.7.0/zephyr/arch/arm/include -I/home/rts/zephyr-2.7.0/zephyr/include -I/home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr/include/generated -I/home/rts/zephyr-2.7.0/zephyr/soc/arm/st_stm32/stm32l4 -I/home/rts/zephyr-2.7.0/zephyr/lib/libc/newlib/include -I/home/rts/zephyr-2.7.0/zephyr/drivers -I/home/rts/zephyr-2.7.0/zephyr/soc/arm/st_stm32/common -I/home/rts/zephyr-2.7.0/modules/hal/cmsis/CMSIS/Core/Include -I/home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/soc -I/home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include -I/home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/Legacy -I/home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/common_ll/include

ASM_FLAGS = -g -Og -imacros /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr/include/generated/autoconf.h -ffreestanding -fno-common -g -gdwarf-4 -fdiagnostics-color=always -mcpu=cortex-m4 -mthumb -mabi=aapcs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mfp16-format=ieee -xassembler-with-cpp -imacros /home/rts/zephyr-2.7.0/zephyr/include/toolchain/zephyr_stdint.h -D_ASMLANGUAGE -Wno-unused-but-set-variable -fno-asynchronous-unwind-tables -fno-pie -fno-pic -fno-strict-overflow -fno-reorder-functions -fno-defer-pop -fmacro-prefix-map=/home/rts/tmp/beacon_project_rzd/app=CMAKE_SOURCE_DIR -fmacro-prefix-map=/home/rts/zephyr-2.7.0/zephyr=ZEPHYR_BASE -fmacro-prefix-map=/home/rts/zephyr-2.7.0=WEST_TOPDIR -ffunction-sections -fdata-sections -specs=nano.specs

C_DEFINES = -DBUILD_VERSION=zephyr-v2.7.0 -DCORE_CM4 -DHSE_VALUE=8000000 -DKERNEL -DSTM32L452xx -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -D_FORTIFY_SOURCE=2 -D__LINUX_ERRNO_EXTENSIONS__ -D__PROGRAM_START -D__ZEPHYR_SUPERVISOR__ -D__ZEPHYR__=1

C_INCLUDES = -I/home/rts/zephyr-2.7.0/zephyr/kernel/include -I/home/rts/zephyr-2.7.0/zephyr/arch/arm/include -I/home/rts/zephyr-2.7.0/zephyr/include -I/home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr/include/generated -I/home/rts/zephyr-2.7.0/zephyr/soc/arm/st_stm32/stm32l4 -I/home/rts/zephyr-2.7.0/zephyr/lib/libc/newlib/include -I/home/rts/zephyr-2.7.0/zephyr/drivers -I/home/rts/zephyr-2.7.0/zephyr/soc/arm/st_stm32/common -I/home/rts/zephyr-2.7.0/modules/hal/cmsis/CMSIS/Core/Include -I/home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/soc -I/home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include -I/home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/Legacy -I/home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/common_ll/include

C_FLAGS = -Og -imacros /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr/include/generated/autoconf.h -ffreestanding -fno-common -g -gdwarf-4 -fdiagnostics-color=always -mcpu=cortex-m4 -mthumb -mabi=aapcs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mfp16-format=ieee -imacros /home/rts/zephyr-2.7.0/zephyr/include/toolchain/zephyr_stdint.h -Wall -Wformat -Wformat-security -Wno-format-zero-length -Wno-main -Wno-pointer-sign -Wpointer-arith -Wexpansion-to-defined -Wno-address-of-packed-member -Wno-unused-but-set-variable -Werror=implicit-int -fno-asynchronous-unwind-tables -fno-pie -fno-pic -fno-strict-overflow -fno-reorder-functions -fno-defer-pop -fmacro-prefix-map=/home/rts/tmp/beacon_project_rzd/app=CMAKE_SOURCE_DIR -fmacro-prefix-map=/home/rts/zephyr-2.7.0/zephyr=ZEPHYR_BASE -fmacro-prefix-map=/home/rts/zephyr-2.7.0=WEST_TOPDIR -ffunction-sections -fdata-sections -specs=nano.specs -std=c99

