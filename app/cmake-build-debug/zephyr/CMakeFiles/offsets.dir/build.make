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
include zephyr/CMakeFiles/offsets.dir/depend.make
# Include the progress variables for this target.
include zephyr/CMakeFiles/offsets.dir/progress.make

# Include the compile flags for this target's objects.
include zephyr/CMakeFiles/offsets.dir/flags.make

zephyr/CMakeFiles/offsets.dir/arch/arm/core/offsets/offsets.c.obj: zephyr/CMakeFiles/offsets.dir/flags.make
zephyr/CMakeFiles/offsets.dir/arch/arm/core/offsets/offsets.c.obj: /home/rts/zephyr-2.7.0/zephyr/arch/arm/core/offsets/offsets.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object zephyr/CMakeFiles/offsets.dir/arch/arm/core/offsets/offsets.c.obj"
	cd /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr && ccache /opt/zephyr-sdk/arm-zephyr-eabi/bin/arm-zephyr-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/offsets.dir/arch/arm/core/offsets/offsets.c.obj -c /home/rts/zephyr-2.7.0/zephyr/arch/arm/core/offsets/offsets.c

zephyr/CMakeFiles/offsets.dir/arch/arm/core/offsets/offsets.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/offsets.dir/arch/arm/core/offsets/offsets.c.i"
	cd /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr && /opt/zephyr-sdk/arm-zephyr-eabi/bin/arm-zephyr-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/rts/zephyr-2.7.0/zephyr/arch/arm/core/offsets/offsets.c > CMakeFiles/offsets.dir/arch/arm/core/offsets/offsets.c.i

zephyr/CMakeFiles/offsets.dir/arch/arm/core/offsets/offsets.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/offsets.dir/arch/arm/core/offsets/offsets.c.s"
	cd /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr && /opt/zephyr-sdk/arm-zephyr-eabi/bin/arm-zephyr-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/rts/zephyr-2.7.0/zephyr/arch/arm/core/offsets/offsets.c -o CMakeFiles/offsets.dir/arch/arm/core/offsets/offsets.c.s

offsets: zephyr/CMakeFiles/offsets.dir/arch/arm/core/offsets/offsets.c.obj
offsets: zephyr/CMakeFiles/offsets.dir/build.make
.PHONY : offsets

# Rule to build all files generated by this target.
zephyr/CMakeFiles/offsets.dir/build: offsets
.PHONY : zephyr/CMakeFiles/offsets.dir/build

zephyr/CMakeFiles/offsets.dir/clean:
	cd /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr && $(CMAKE_COMMAND) -P CMakeFiles/offsets.dir/cmake_clean.cmake
.PHONY : zephyr/CMakeFiles/offsets.dir/clean

zephyr/CMakeFiles/offsets.dir/depend:
	cd /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rts/tmp/beacon_project_rzd/app /home/rts/zephyr-2.7.0/zephyr /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr/CMakeFiles/offsets.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : zephyr/CMakeFiles/offsets.dir/depend

