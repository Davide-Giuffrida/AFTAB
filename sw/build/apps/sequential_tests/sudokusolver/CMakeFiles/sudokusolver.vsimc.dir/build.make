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

# Utility rule file for sudokusolver.vsimc.

# Include the progress variables for this target.
include apps/sequential_tests/sudokusolver/CMakeFiles/sudokusolver.vsimc.dir/progress.make

apps/sequential_tests/sudokusolver/CMakeFiles/sudokusolver.vsimc:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/user/Desktop/Thesis/aftab/sw/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Running sudokusolver in ModelSim"
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/sequential_tests/sudokusolver && /usr/bin/cmake -E remove stdout/*
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/sequential_tests/sudokusolver && /usr/bin/cmake -E remove FS/*
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/sequential_tests/sudokusolver && tcsh -c env\ VSIM_DIR=/home/user/Desktop/Thesis/aftab/vsim\ USE_ZERO_RISCY=1\ RISCY_RV32F=0\ ZERO_RV32M=1\ ZERO_RV32E=0\ PL_NETLIST=\ TB_TEST=""\ /usr/local/modelsim/modelsim_ase/bin/vsim\ \ -c\ -64\ -do\ 'source\ tcl_files/run.tcl\;\ run\ -a\;\ exit\;'\ >vsim.log
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/sequential_tests/sudokusolver && tail -n +1 -- ./stdout/*

sudokusolver.vsimc: apps/sequential_tests/sudokusolver/CMakeFiles/sudokusolver.vsimc
sudokusolver.vsimc: apps/sequential_tests/sudokusolver/CMakeFiles/sudokusolver.vsimc.dir/build.make

.PHONY : sudokusolver.vsimc

# Rule to build all files generated by this target.
apps/sequential_tests/sudokusolver/CMakeFiles/sudokusolver.vsimc.dir/build: sudokusolver.vsimc

.PHONY : apps/sequential_tests/sudokusolver/CMakeFiles/sudokusolver.vsimc.dir/build

apps/sequential_tests/sudokusolver/CMakeFiles/sudokusolver.vsimc.dir/clean:
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/sequential_tests/sudokusolver && $(CMAKE_COMMAND) -P CMakeFiles/sudokusolver.vsimc.dir/cmake_clean.cmake
.PHONY : apps/sequential_tests/sudokusolver/CMakeFiles/sudokusolver.vsimc.dir/clean

apps/sequential_tests/sudokusolver/CMakeFiles/sudokusolver.vsimc.dir/depend:
	cd /home/user/Desktop/Thesis/aftab/sw/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/Desktop/Thesis/aftab/sw /home/user/Desktop/Thesis/aftab/sw/apps/sequential_tests/sudokusolver /home/user/Desktop/Thesis/aftab/sw/build /home/user/Desktop/Thesis/aftab/sw/build/apps/sequential_tests/sudokusolver /home/user/Desktop/Thesis/aftab/sw/build/apps/sequential_tests/sudokusolver/CMakeFiles/sudokusolver.vsimc.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : apps/sequential_tests/sudokusolver/CMakeFiles/sudokusolver.vsimc.dir/depend

