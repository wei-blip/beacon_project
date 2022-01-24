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
include zephyr/arch/common/CMakeFiles/isr_tables.dir/depend.make
# Include the progress variables for this target.
include zephyr/arch/common/CMakeFiles/isr_tables.dir/progress.make

# Include the compile flags for this target's objects.
include zephyr/arch/common/CMakeFiles/isr_tables.dir/flags.make

zephyr/arch/common/CMakeFiles/isr_tables.dir/isr_tables.c.obj: zephyr/arch/common/CMakeFiles/isr_tables.dir/flags.make
zephyr/arch/common/CMakeFiles/isr_tables.dir/isr_tables.c.obj: /home/rts/zephyr-2.7.0/zephyr/arch/common/isr_tables.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object zephyr/arch/common/CMakeFiles/isr_tables.dir/isr_tables.c.obj"
	cd /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr/arch/common && ccache /opt/zephyr-sdk/arm-zephyr-eabi/bin/arm-zephyr-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/isr_tables.dir/isr_tables.c.obj -c /home/rts/zephyr-2.7.0/zephyr/arch/common/isr_tables.c

zephyr/arch/common/CMakeFiles/isr_tables.dir/isr_tables.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/isr_tables.dir/isr_tables.c.i"
	cd /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr/arch/common && /opt/zephyr-sdk/arm-zephyr-eabi/bin/arm-zephyr-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/rts/zephyr-2.7.0/zephyr/arch/common/isr_tables.c > CMakeFiles/isr_tables.dir/isr_tables.c.i

zephyr/arch/common/CMakeFiles/isr_tables.dir/isr_tables.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/isr_tables.dir/isr_tables.c.s"
	cd /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr/arch/common && /opt/zephyr-sdk/arm-zephyr-eabi/bin/arm-zephyr-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/rts/zephyr-2.7.0/zephyr/arch/common/isr_tables.c -o CMakeFiles/isr_tables.dir/isr_tables.c.s

# Object files for target isr_tables
isr_tables_OBJECTS = \
"CMakeFiles/isr_tables.dir/isr_tables.c.obj"

# External object files for target isr_tables
isr_tables_EXTERNAL_OBJECTS =

zephyr/arch/common/libisr_tables.a: zephyr/arch/common/CMakeFiles/isr_tables.dir/isr_tables.c.obj
zephyr/arch/common/libisr_tables.a: zephyr/arch/common/CMakeFiles/isr_tables.dir/build.make
zephyr/arch/common/libisr_tables.a: zephyr/arch/common/CMakeFiles/isr_tables.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C static library libisr_tables.a"
	cd /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr/arch/common && $(CMAKE_COMMAND) -P CMakeFiles/isr_tables.dir/cmake_clean_target.cmake
	cd /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr/arch/common && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/isr_tables.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
zephyr/arch/common/CMakeFiles/isr_tables.dir/build: zephyr/arch/common/libisr_tables.a
.PHONY : zephyr/arch/common/CMakeFiles/isr_tables.dir/build

zephyr/arch/common/CMakeFiles/isr_tables.dir/clean:
	cd /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr/arch/common && $(CMAKE_COMMAND) -P CMakeFiles/isr_tables.dir/cmake_clean.cmake
.PHONY : zephyr/arch/common/CMakeFiles/isr_tables.dir/clean

zephyr/arch/common/CMakeFiles/isr_tables.dir/depend:
	cd /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rts/tmp/beacon_project_rzd/app /home/rts/zephyr-2.7.0/zephyr/arch/common /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr/arch/common /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr/arch/common/CMakeFiles/isr_tables.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : zephyr/arch/common/CMakeFiles/isr_tables.dir/depend

