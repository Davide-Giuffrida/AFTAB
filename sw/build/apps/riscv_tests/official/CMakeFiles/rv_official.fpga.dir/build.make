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

# Utility rule file for rv_official.fpga.

# Include the progress variables for this target.
include apps/riscv_tests/official/CMakeFiles/rv_official.fpga.dir/progress.make

apps/riscv_tests/official/CMakeFiles/rv_official.fpga:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/user/Desktop/Thesis/aftab/sw/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Running rv_official on FPGA"
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/riscv_tests/official && /home/user/Desktop/Thesis/aftab/sw/utils/run-on-fpga.sh rv_official

rv_official.fpga: apps/riscv_tests/official/CMakeFiles/rv_official.fpga
rv_official.fpga: apps/riscv_tests/official/CMakeFiles/rv_official.fpga.dir/build.make

.PHONY : rv_official.fpga

# Rule to build all files generated by this target.
apps/riscv_tests/official/CMakeFiles/rv_official.fpga.dir/build: rv_official.fpga

.PHONY : apps/riscv_tests/official/CMakeFiles/rv_official.fpga.dir/build

apps/riscv_tests/official/CMakeFiles/rv_official.fpga.dir/clean:
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/riscv_tests/official && $(CMAKE_COMMAND) -P CMakeFiles/rv_official.fpga.dir/cmake_clean.cmake
.PHONY : apps/riscv_tests/official/CMakeFiles/rv_official.fpga.dir/clean

apps/riscv_tests/official/CMakeFiles/rv_official.fpga.dir/depend:
	cd /home/user/Desktop/Thesis/aftab/sw/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/Desktop/Thesis/aftab/sw /home/user/Desktop/Thesis/aftab/sw/apps/riscv_tests/official /home/user/Desktop/Thesis/aftab/sw/build /home/user/Desktop/Thesis/aftab/sw/build/apps/riscv_tests/official /home/user/Desktop/Thesis/aftab/sw/build/apps/riscv_tests/official/CMakeFiles/rv_official.fpga.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : apps/riscv_tests/official/CMakeFiles/rv_official.fpga.dir/depend

