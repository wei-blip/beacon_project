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

# Utility rule file for syscall_list_h_target.

# Include any custom commands dependencies for this target.
include zephyr/CMakeFiles/syscall_list_h_target.dir/compiler_depend.make

# Include the progress variables for this target.
include zephyr/CMakeFiles/syscall_list_h_target.dir/progress.make

zephyr/CMakeFiles/syscall_list_h_target: zephyr/include/generated/syscall_list.h

zephyr/include/generated/syscall_dispatch.c:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating include/generated/syscall_dispatch.c, include/generated/syscall_list.h"
	cd /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr && /usr/bin/python3.9 /home/rts/zephyr-2.7.0/zephyr/scripts/gen_syscalls.py --json-file /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr/misc/generated/syscalls.json --base-output include/generated/syscalls --syscall-dispatch include/generated/syscall_dispatch.c --syscall-list /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr/include/generated/syscall_list.h --split-type k_timeout_t

zephyr/include/generated/syscall_list.h: zephyr/include/generated/syscall_dispatch.c
	@$(CMAKE_COMMAND) -E touch_nocreate zephyr/include/generated/syscall_list.h

syscall_list_h_target: zephyr/CMakeFiles/syscall_list_h_target
syscall_list_h_target: zephyr/include/generated/syscall_dispatch.c
syscall_list_h_target: zephyr/include/generated/syscall_list.h
syscall_list_h_target: zephyr/CMakeFiles/syscall_list_h_target.dir/build.make
.PHONY : syscall_list_h_target

# Rule to build all files generated by this target.
zephyr/CMakeFiles/syscall_list_h_target.dir/build: syscall_list_h_target
.PHONY : zephyr/CMakeFiles/syscall_list_h_target.dir/build

zephyr/CMakeFiles/syscall_list_h_target.dir/clean:
	cd /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr && $(CMAKE_COMMAND) -P CMakeFiles/syscall_list_h_target.dir/cmake_clean.cmake
.PHONY : zephyr/CMakeFiles/syscall_list_h_target.dir/clean

zephyr/CMakeFiles/syscall_list_h_target.dir/depend:
	cd /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rts/tmp/beacon_project_rzd/app /home/rts/zephyr-2.7.0/zephyr /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr /home/rts/tmp/beacon_project_rzd/app/cmake-build-debug/zephyr/CMakeFiles/syscall_list_h_target.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : zephyr/CMakeFiles/syscall_list_h_target.dir/depend

