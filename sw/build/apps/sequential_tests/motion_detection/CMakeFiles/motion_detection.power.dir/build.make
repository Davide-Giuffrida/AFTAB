# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
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
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/user/Desktop/Thesis/aftab/sw

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/user/Desktop/Thesis/aftab/sw/build

# Utility rule file for motion_detection.power.

# Include the progress variables for this target.
include apps/sequential_tests/motion_detection/CMakeFiles/motion_detection.power.dir/progress.make

apps/sequential_tests/motion_detection/CMakeFiles/motion_detection.power:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/user/Desktop/Thesis/aftab/sw/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Running motion_detection in ModelSim (post layout)"
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/sequential_tests/motion_detection && /usr/bin/cmake -E remove stdout/*
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/sequential_tests/motion_detection && /usr/bin/cmake -E remove FS/*
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/sequential_tests/motion_detection && tcsh -c env\ VSIM_DIR=/home/user/Desktop/Thesis/aftab/vsim\ USE_ZERO_RISCY=1\ RISCY_RV32F=0\ ZERO_RV32M=1\ ZERO_RV32E=0\ PL_NETLIST=\ TB_TEST=""\ /usr/local/modelsim/modelsim_ase/bin/vsim\ \ -64\ -do\ 'source\ tcl_files/run_power.tcl\;'

motion_detection.power: apps/sequential_tests/motion_detection/CMakeFiles/motion_detection.power
motion_detection.power: apps/sequential_tests/motion_detection/CMakeFiles/motion_detection.power.dir/build.make

.PHONY : motion_detection.power

# Rule to build all files generated by this target.
apps/sequential_tests/motion_detection/CMakeFiles/motion_detection.power.dir/build: motion_detection.power

.PHONY : apps/sequential_tests/motion_detection/CMakeFiles/motion_detection.power.dir/build

apps/sequential_tests/motion_detection/CMakeFiles/motion_detection.power.dir/clean:
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/sequential_tests/motion_detection && $(CMAKE_COMMAND) -P CMakeFiles/motion_detection.power.dir/cmake_clean.cmake
.PHONY : apps/sequential_tests/motion_detection/CMakeFiles/motion_detection.power.dir/clean

apps/sequential_tests/motion_detection/CMakeFiles/motion_detection.power.dir/depend:
	cd /home/user/Desktop/Thesis/aftab/sw/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/Desktop/Thesis/aftab/sw /home/user/Desktop/Thesis/aftab/sw/apps/sequential_tests/motion_detection /home/user/Desktop/Thesis/aftab/sw/build /home/user/Desktop/Thesis/aftab/sw/build/apps/sequential_tests/motion_detection /home/user/Desktop/Thesis/aftab/sw/build/apps/sequential_tests/motion_detection/CMakeFiles/motion_detection.power.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : apps/sequential_tests/motion_detection/CMakeFiles/motion_detection.power.dir/depend

