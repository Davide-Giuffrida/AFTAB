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

# Utility rule file for crc32.annotate.

# Include the progress variables for this target.
include apps/sequential_tests/crc32/CMakeFiles/crc32.annotate.dir/progress.make

crc32.annotate: apps/sequential_tests/crc32/CMakeFiles/crc32.annotate.dir/build.make
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/sequential_tests/crc32 && /home/user/Desktop/Thesis/aftab/sw/utils/annotate.py crc32.read
.PHONY : crc32.annotate

# Rule to build all files generated by this target.
apps/sequential_tests/crc32/CMakeFiles/crc32.annotate.dir/build: crc32.annotate

.PHONY : apps/sequential_tests/crc32/CMakeFiles/crc32.annotate.dir/build

apps/sequential_tests/crc32/CMakeFiles/crc32.annotate.dir/clean:
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/sequential_tests/crc32 && $(CMAKE_COMMAND) -P CMakeFiles/crc32.annotate.dir/cmake_clean.cmake
.PHONY : apps/sequential_tests/crc32/CMakeFiles/crc32.annotate.dir/clean

apps/sequential_tests/crc32/CMakeFiles/crc32.annotate.dir/depend:
	cd /home/user/Desktop/Thesis/aftab/sw/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/Desktop/Thesis/aftab/sw /home/user/Desktop/Thesis/aftab/sw/apps/sequential_tests/crc32 /home/user/Desktop/Thesis/aftab/sw/build /home/user/Desktop/Thesis/aftab/sw/build/apps/sequential_tests/crc32 /home/user/Desktop/Thesis/aftab/sw/build/apps/sequential_tests/crc32/CMakeFiles/crc32.annotate.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : apps/sequential_tests/crc32/CMakeFiles/crc32.annotate.dir/depend

