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

# Utility rule file for testEvents.read.

# Include the progress variables for this target.
include apps/riscv_tests/testEvents/CMakeFiles/testEvents.read.dir/progress.make

testEvents.read: apps/riscv_tests/testEvents/CMakeFiles/testEvents.read.dir/build.make
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/riscv_tests/testEvents && /opt/riscv/bin/riscv32-unknown-elf-objdump -Mmarch=rv32im -d /home/user/Desktop/Thesis/aftab/sw/build/apps/riscv_tests/testEvents/testEvents.elf > testEvents.read
.PHONY : testEvents.read

# Rule to build all files generated by this target.
apps/riscv_tests/testEvents/CMakeFiles/testEvents.read.dir/build: testEvents.read

.PHONY : apps/riscv_tests/testEvents/CMakeFiles/testEvents.read.dir/build

apps/riscv_tests/testEvents/CMakeFiles/testEvents.read.dir/clean:
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/riscv_tests/testEvents && $(CMAKE_COMMAND) -P CMakeFiles/testEvents.read.dir/cmake_clean.cmake
.PHONY : apps/riscv_tests/testEvents/CMakeFiles/testEvents.read.dir/clean

apps/riscv_tests/testEvents/CMakeFiles/testEvents.read.dir/depend:
	cd /home/user/Desktop/Thesis/aftab/sw/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/Desktop/Thesis/aftab/sw /home/user/Desktop/Thesis/aftab/sw/apps/riscv_tests/testEvents /home/user/Desktop/Thesis/aftab/sw/build /home/user/Desktop/Thesis/aftab/sw/build/apps/riscv_tests/testEvents /home/user/Desktop/Thesis/aftab/sw/build/apps/riscv_tests/testEvents/CMakeFiles/testEvents.read.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : apps/riscv_tests/testEvents/CMakeFiles/testEvents.read.dir/depend

