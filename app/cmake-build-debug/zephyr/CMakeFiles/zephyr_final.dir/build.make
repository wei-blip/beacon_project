# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.21

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/rts/tmp/beacon_project_rzd/app

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug

# Include any dependencies generated for this target.
include zephyr/CMakeFiles/zephyr_final.dir/depend.make
# Include the progress variables for this target.
include zephyr/CMakeFiles/zephyr_final.dir/progress.make

# Include the compile flags for this target's objects.
include zephyr/CMakeFiles/zephyr_final.dir/flags.make

zephyr/isr_tables.c: zephyr/zephyr_prebuilt.elf
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating isr_tables.c, isrList.bin"
	cd /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr && /opt/zephyr-sdk/arm-zephyr-eabi/bin/arm-zephyr-eabi-objcopy --input-target=elf32-littlearm --output-target=binary --only-section=.intList /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr/zephyr_prebuilt.elf isrList.bin
	cd /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr && /usr/bin/python3.9 /home/rts/zephyr-2.7.0/zephyr/arch/common/gen_isr_tables.py --output-source isr_tables.c --kernel /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr/zephyr_prebuilt.elf --intlist isrList.bin --sw-isr-table --vector-table

zephyr/isrList.bin: zephyr/isr_tables.c
	@$(CMAKE_COMMAND) -E touch_nocreate zephyr/isrList.bin

zephyr/dev_handles.c: zephyr/zephyr_prebuilt.elf
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating dev_handles.c"
	cd /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr && /usr/bin/python3.9 /home/rts/zephyr-2.7.0/zephyr/scripts/gen_handles.py --output-source dev_handles.c --kernel /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr/zephyr_prebuilt.elf --zephyr-base /home/rts/zephyr-2.7.0/zephyr --start-symbol __device_start

zephyr/CMakeFiles/zephyr_final.dir/misc/empty_file.c.obj: zephyr/CMakeFiles/zephyr_final.dir/flags.make
zephyr/CMakeFiles/zephyr_final.dir/misc/empty_file.c.obj: /home/rts/zephyr-2.7.0/zephyr/misc/empty_file.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object zephyr/CMakeFiles/zephyr_final.dir/misc/empty_file.c.obj"
	cd /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr && ccache /opt/zephyr-sdk/arm-zephyr-eabi/bin/arm-zephyr-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/zephyr_final.dir/misc/empty_file.c.obj -c /home/rts/zephyr-2.7.0/zephyr/misc/empty_file.c

zephyr/CMakeFiles/zephyr_final.dir/misc/empty_file.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/zephyr_final.dir/misc/empty_file.c.i"
	cd /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr && /opt/zephyr-sdk/arm-zephyr-eabi/bin/arm-zephyr-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/rts/zephyr-2.7.0/zephyr/misc/empty_file.c > CMakeFiles/zephyr_final.dir/misc/empty_file.c.i

zephyr/CMakeFiles/zephyr_final.dir/misc/empty_file.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/zephyr_final.dir/misc/empty_file.c.s"
	cd /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr && /opt/zephyr-sdk/arm-zephyr-eabi/bin/arm-zephyr-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/rts/zephyr-2.7.0/zephyr/misc/empty_file.c -o CMakeFiles/zephyr_final.dir/misc/empty_file.c.s

zephyr/CMakeFiles/zephyr_final.dir/isr_tables.c.obj: zephyr/CMakeFiles/zephyr_final.dir/flags.make
zephyr/CMakeFiles/zephyr_final.dir/isr_tables.c.obj: zephyr/isr_tables.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object zephyr/CMakeFiles/zephyr_final.dir/isr_tables.c.obj"
	cd /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr && ccache /opt/zephyr-sdk/arm-zephyr-eabi/bin/arm-zephyr-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/zephyr_final.dir/isr_tables.c.obj -c /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr/isr_tables.c

zephyr/CMakeFiles/zephyr_final.dir/isr_tables.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/zephyr_final.dir/isr_tables.c.i"
	cd /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr && /opt/zephyr-sdk/arm-zephyr-eabi/bin/arm-zephyr-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr/isr_tables.c > CMakeFiles/zephyr_final.dir/isr_tables.c.i

zephyr/CMakeFiles/zephyr_final.dir/isr_tables.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/zephyr_final.dir/isr_tables.c.s"
	cd /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr && /opt/zephyr-sdk/arm-zephyr-eabi/bin/arm-zephyr-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr/isr_tables.c -o CMakeFiles/zephyr_final.dir/isr_tables.c.s

zephyr/CMakeFiles/zephyr_final.dir/dev_handles.c.obj: zephyr/CMakeFiles/zephyr_final.dir/flags.make
zephyr/CMakeFiles/zephyr_final.dir/dev_handles.c.obj: zephyr/dev_handles.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object zephyr/CMakeFiles/zephyr_final.dir/dev_handles.c.obj"
	cd /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr && ccache /opt/zephyr-sdk/arm-zephyr-eabi/bin/arm-zephyr-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/zephyr_final.dir/dev_handles.c.obj -c /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr/dev_handles.c

zephyr/CMakeFiles/zephyr_final.dir/dev_handles.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/zephyr_final.dir/dev_handles.c.i"
	cd /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr && /opt/zephyr-sdk/arm-zephyr-eabi/bin/arm-zephyr-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr/dev_handles.c > CMakeFiles/zephyr_final.dir/dev_handles.c.i

zephyr/CMakeFiles/zephyr_final.dir/dev_handles.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/zephyr_final.dir/dev_handles.c.s"
	cd /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr && /opt/zephyr-sdk/arm-zephyr-eabi/bin/arm-zephyr-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr/dev_handles.c -o CMakeFiles/zephyr_final.dir/dev_handles.c.s

# Object files for target zephyr_final
zephyr_final_OBJECTS = \
"CMakeFiles/zephyr_final.dir/misc/empty_file.c.obj" \
"CMakeFiles/zephyr_final.dir/isr_tables.c.obj" \
"CMakeFiles/zephyr_final.dir/dev_handles.c.obj"

# External object files for target zephyr_final
zephyr_final_EXTERNAL_OBJECTS =

zephyr/zephyr.elf: zephyr/CMakeFiles/zephyr_final.dir/misc/empty_file.c.obj
zephyr/zephyr.elf: zephyr/CMakeFiles/zephyr_final.dir/isr_tables.c.obj
zephyr/zephyr.elf: zephyr/CMakeFiles/zephyr_final.dir/dev_handles.c.obj
zephyr/zephyr.elf: zephyr/CMakeFiles/zephyr_final.dir/build.make
zephyr/zephyr.elf: zephyr/CMakeFiles/offsets.dir/arch/arm/core/offsets/offsets.c.obj
zephyr/zephyr.elf: zephyr/linker.cmd
zephyr/zephyr.elf: app/libapp.a
zephyr/zephyr.elf: zephyr/libzephyr.a
zephyr/zephyr.elf: zephyr/arch/common/libarch__common.a
zephyr/zephyr.elf: zephyr/arch/arch/arm/core/aarch32/libarch__arm__core__aarch32.a
zephyr/zephyr.elf: zephyr/arch/arch/arm/core/aarch32/cortex_m/libarch__arm__core__aarch32__cortex_m.a
zephyr/zephyr.elf: zephyr/arch/arch/arm/core/aarch32/mpu/libarch__arm__core__aarch32__mpu.a
zephyr/zephyr.elf: zephyr/lib/libc/newlib/liblib__libc__newlib.a
zephyr/zephyr.elf: zephyr/lib/posix/liblib__posix.a
zephyr/zephyr.elf: zephyr/soc/arm/common/cortex_m/libsoc__arm__common__cortex_m.a
zephyr/zephyr.elf: zephyr/drivers/interrupt_controller/libdrivers__interrupt_controller.a
zephyr/zephyr.elf: zephyr/drivers/clock_control/libdrivers__clock_control.a
zephyr/zephyr.elf: zephyr/drivers/console/libdrivers__console.a
zephyr/zephyr.elf: zephyr/drivers/dma/libdrivers__dma.a
zephyr/zephyr.elf: zephyr/drivers/gpio/libdrivers__gpio.a
zephyr/zephyr.elf: zephyr/drivers/spi/libdrivers__spi.a
zephyr/zephyr.elf: zephyr/drivers/serial/libdrivers__serial.a
zephyr/zephyr.elf: zephyr/drivers/timer/libdrivers__timer.a
zephyr/zephyr.elf: modules/stm32/stm32cube/lib..__modules__hal__stm32__stm32cube.a
zephyr/zephyr.elf: zephyr/kernel/libkernel.a
zephyr/zephyr.elf: zephyr/arch/common/libisr_tables.a
zephyr/zephyr.elf: zephyr/linker.cmd
zephyr/zephyr.elf: zephyr/CMakeFiles/zephyr_final.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking C executable zephyr.elf"
	cd /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/zephyr_final.dir/link.txt --verbose=$(VERBOSE)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating files from zephyr.elf for board: rs_0022_0"
	cd /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr && /usr/bin/cmake -E rename zephyr_final.map zephyr.map
	cd /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr && /opt/zephyr-sdk/arm-zephyr-eabi/bin/arm-zephyr-eabi-objcopy --gap-fill 0xff --output-target=ihex --remove-section=.comment --remove-section=COMMON --remove-section=.eh_frame zephyr.elf zephyr.hex
	cd /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr && /opt/zephyr-sdk/arm-zephyr-eabi/bin/arm-zephyr-eabi-objcopy --gap-fill 0xff --output-target=binary --remove-section=.comment --remove-section=COMMON --remove-section=.eh_frame zephyr.elf zephyr.bin
	cd /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr && /opt/zephyr-sdk/arm-zephyr-eabi/bin/arm-zephyr-eabi-objdump -d -S zephyr.elf > zephyr.lst
	cd /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr && /opt/zephyr-sdk/arm-zephyr-eabi/bin/arm-zephyr-eabi-readelf -e zephyr.elf > zephyr.stat

# Rule to build all files generated by this target.
zephyr/CMakeFiles/zephyr_final.dir/build: zephyr/zephyr.elf
.PHONY : zephyr/CMakeFiles/zephyr_final.dir/build

zephyr/CMakeFiles/zephyr_final.dir/clean:
	cd /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr && $(CMAKE_COMMAND) -P CMakeFiles/zephyr_final.dir/cmake_clean.cmake
.PHONY : zephyr/CMakeFiles/zephyr_final.dir/clean

zephyr/CMakeFiles/zephyr_final.dir/depend: zephyr/dev_handles.c
zephyr/CMakeFiles/zephyr_final.dir/depend: zephyr/isrList.bin
zephyr/CMakeFiles/zephyr_final.dir/depend: zephyr/isr_tables.c
	cd /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rts/tmp/beacon_project_rzd/app /home/rts/zephyr-2.7.0/zephyr /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr/CMakeFiles/zephyr_final.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : zephyr/CMakeFiles/zephyr_final.dir/depend

