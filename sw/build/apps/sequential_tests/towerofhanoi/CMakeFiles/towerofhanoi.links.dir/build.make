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

# Utility rule file for towerofhanoi.links.

# Include the progress variables for this target.
include apps/sequential_tests/towerofhanoi/CMakeFiles/towerofhanoi.links.dir/progress.make

apps/sequential_tests/towerofhanoi/CMakeFiles/towerofhanoi.links: apps/sequential_tests/towerofhanoi/modelsim.ini
apps/sequential_tests/towerofhanoi/CMakeFiles/towerofhanoi.links: apps/sequential_tests/towerofhanoi/work
apps/sequential_tests/towerofhanoi/CMakeFiles/towerofhanoi.links: apps/sequential_tests/towerofhanoi/tcl_files
apps/sequential_tests/towerofhanoi/CMakeFiles/towerofhanoi.links: apps/sequential_tests/towerofhanoi/waves


apps/sequential_tests/towerofhanoi/modelsim.ini:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/user/Desktop/Thesis/aftab/sw/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating modelsim.ini"
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/sequential_tests/towerofhanoi && /usr/bin/cmake -E create_symlink /home/user/Desktop/Thesis/aftab/vsim/modelsim.ini /home/user/Desktop/Thesis/aftab/sw/build/apps/sequential_tests/towerofhanoi//modelsim.ini

apps/sequential_tests/towerofhanoi/work:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/user/Desktop/Thesis/aftab/sw/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating work"
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/sequential_tests/towerofhanoi && /usr/bin/cmake -E create_symlink /home/user/Desktop/Thesis/aftab/vsim/work /home/user/Desktop/Thesis/aftab/sw/build/apps/sequential_tests/towerofhanoi//work

apps/sequential_tests/towerofhanoi/tcl_files:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/user/Desktop/Thesis/aftab/sw/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Generating tcl_files"
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/sequential_tests/towerofhanoi && /usr/bin/cmake -E create_symlink /home/user/Desktop/Thesis/aftab/vsim/tcl_files /home/user/Desktop/Thesis/aftab/sw/build/apps/sequential_tests/towerofhanoi//tcl_files

apps/sequential_tests/towerofhanoi/waves:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/user/Desktop/Thesis/aftab/sw/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Generating waves"
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/sequential_tests/towerofhanoi && /usr/bin/cmake -E create_symlink /home/user/Desktop/Thesis/aftab/vsim/waves /home/user/Desktop/Thesis/aftab/sw/build/apps/sequential_tests/towerofhanoi//waves

towerofhanoi.links: apps/sequential_tests/towerofhanoi/CMakeFiles/towerofhanoi.links
towerofhanoi.links: apps/sequential_tests/towerofhanoi/modelsim.ini
towerofhanoi.links: apps/sequential_tests/towerofhanoi/work
towerofhanoi.links: apps/sequential_tests/towerofhanoi/tcl_files
towerofhanoi.links: apps/sequential_tests/towerofhanoi/waves
towerofhanoi.links: apps/sequential_tests/towerofhanoi/CMakeFiles/towerofhanoi.links.dir/build.make

.PHONY : towerofhanoi.links

# Rule to build all files generated by this target.
apps/sequential_tests/towerofhanoi/CMakeFiles/towerofhanoi.links.dir/build: towerofhanoi.links

.PHONY : apps/sequential_tests/towerofhanoi/CMakeFiles/towerofhanoi.links.dir/build

apps/sequential_tests/towerofhanoi/CMakeFiles/towerofhanoi.links.dir/clean:
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/sequential_tests/towerofhanoi && $(CMAKE_COMMAND) -P CMakeFiles/towerofhanoi.links.dir/cmake_clean.cmake
.PHONY : apps/sequential_tests/towerofhanoi/CMakeFiles/towerofhanoi.links.dir/clean

apps/sequential_tests/towerofhanoi/CMakeFiles/towerofhanoi.links.dir/depend:
	cd /home/user/Desktop/Thesis/aftab/sw/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/Desktop/Thesis/aftab/sw /home/user/Desktop/Thesis/aftab/sw/apps/sequential_tests/towerofhanoi /home/user/Desktop/Thesis/aftab/sw/build /home/user/Desktop/Thesis/aftab/sw/build/apps/sequential_tests/towerofhanoi /home/user/Desktop/Thesis/aftab/sw/build/apps/sequential_tests/towerofhanoi/CMakeFiles/towerofhanoi.links.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : apps/sequential_tests/towerofhanoi/CMakeFiles/towerofhanoi.links.dir/depend

