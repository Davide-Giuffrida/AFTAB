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
CMAKE_SOURCE_DIR = /home/user/Desktop/Thesis/mc2101/sw

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/user/Desktop/Thesis/mc2101/sw/build

# Utility rule file for cnn_layer.vsim.

# Include the progress variables for this target.
include apps/sequential_tests/cnn_layer/CMakeFiles/cnn_layer.vsim.dir/progress.make

apps/sequential_tests/cnn_layer/CMakeFiles/cnn_layer.vsim:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/user/Desktop/Thesis/mc2101/sw/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Running cnn_layer in ModelSim"
	cd /home/user/Desktop/Thesis/mc2101/sw/build/apps/sequential_tests/cnn_layer && /usr/bin/cmake -E remove stdout/*
	cd /home/user/Desktop/Thesis/mc2101/sw/build/apps/sequential_tests/cnn_layer && /usr/bin/cmake -E remove FS/*
	cd /home/user/Desktop/Thesis/mc2101/sw/build/apps/sequential_tests/cnn_layer && tcsh -c env\ VSIM_DIR=/home/user/Desktop/Thesis/mc2101/vsim\ USE_ZERO_RISCY=1\ RISCY_RV32F=0\ ZERO_RV32M=1\ ZERO_RV32E=0\ PL_NETLIST=\ TB_TEST=""\ /usr/local/modelsim/modelsim_ase/bin/vsim\ \ -64\ -do\ 'source\ tcl_files/run.tcl\;'

cnn_layer.vsim: apps/sequential_tests/cnn_layer/CMakeFiles/cnn_layer.vsim
cnn_layer.vsim: apps/sequential_tests/cnn_layer/CMakeFiles/cnn_layer.vsim.dir/build.make

.PHONY : cnn_layer.vsim

# Rule to build all files generated by this target.
apps/sequential_tests/cnn_layer/CMakeFiles/cnn_layer.vsim.dir/build: cnn_layer.vsim

.PHONY : apps/sequential_tests/cnn_layer/CMakeFiles/cnn_layer.vsim.dir/build

apps/sequential_tests/cnn_layer/CMakeFiles/cnn_layer.vsim.dir/clean:
	cd /home/user/Desktop/Thesis/mc2101/sw/build/apps/sequential_tests/cnn_layer && $(CMAKE_COMMAND) -P CMakeFiles/cnn_layer.vsim.dir/cmake_clean.cmake
.PHONY : apps/sequential_tests/cnn_layer/CMakeFiles/cnn_layer.vsim.dir/clean

apps/sequential_tests/cnn_layer/CMakeFiles/cnn_layer.vsim.dir/depend:
	cd /home/user/Desktop/Thesis/mc2101/sw/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/Desktop/Thesis/mc2101/sw /home/user/Desktop/Thesis/mc2101/sw/apps/sequential_tests/cnn_layer /home/user/Desktop/Thesis/mc2101/sw/build /home/user/Desktop/Thesis/mc2101/sw/build/apps/sequential_tests/cnn_layer /home/user/Desktop/Thesis/mc2101/sw/build/apps/sequential_tests/cnn_layer/CMakeFiles/cnn_layer.vsim.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : apps/sequential_tests/cnn_layer/CMakeFiles/cnn_layer.vsim.dir/depend

