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

# Utility rule file for test_asm.fpga.

# Include the progress variables for this target.
include apps/user_apps/test_asm/CMakeFiles/test_asm.fpga.dir/progress.make

apps/user_apps/test_asm/CMakeFiles/test_asm.fpga:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/user/Desktop/Thesis/aftab/sw/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Running test_asm on FPGA"
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/user_apps/test_asm && /home/user/Desktop/Thesis/aftab/sw/utils/run-on-fpga.sh test_asm

test_asm.fpga: apps/user_apps/test_asm/CMakeFiles/test_asm.fpga
test_asm.fpga: apps/user_apps/test_asm/CMakeFiles/test_asm.fpga.dir/build.make

.PHONY : test_asm.fpga

# Rule to build all files generated by this target.
apps/user_apps/test_asm/CMakeFiles/test_asm.fpga.dir/build: test_asm.fpga

.PHONY : apps/user_apps/test_asm/CMakeFiles/test_asm.fpga.dir/build

apps/user_apps/test_asm/CMakeFiles/test_asm.fpga.dir/clean:
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/user_apps/test_asm && $(CMAKE_COMMAND) -P CMakeFiles/test_asm.fpga.dir/cmake_clean.cmake
.PHONY : apps/user_apps/test_asm/CMakeFiles/test_asm.fpga.dir/clean

apps/user_apps/test_asm/CMakeFiles/test_asm.fpga.dir/depend:
	cd /home/user/Desktop/Thesis/aftab/sw/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/Desktop/Thesis/aftab/sw /home/user/Desktop/Thesis/aftab/sw/apps/user_apps/test_asm /home/user/Desktop/Thesis/aftab/sw/build /home/user/Desktop/Thesis/aftab/sw/build/apps/user_apps/test_asm /home/user/Desktop/Thesis/aftab/sw/build/apps/user_apps/test_asm/CMakeFiles/test_asm.fpga.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : apps/user_apps/test_asm/CMakeFiles/test_asm.fpga.dir/depend

