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

# Utility rule file for testInterrupt.kcg.

# Include the progress variables for this target.
include apps/imperio_tests/testInterrupt/CMakeFiles/testInterrupt.kcg.dir/progress.make

apps/imperio_tests/testInterrupt/CMakeFiles/testInterrupt.kcg:
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/imperio_tests/testInterrupt && pulp-pc-analyze --rtl --input=trace_core_00.log --binary=testInterrupt.elf
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/imperio_tests/testInterrupt && kcachegrind kcg.txt

testInterrupt.kcg: apps/imperio_tests/testInterrupt/CMakeFiles/testInterrupt.kcg
testInterrupt.kcg: apps/imperio_tests/testInterrupt/CMakeFiles/testInterrupt.kcg.dir/build.make

.PHONY : testInterrupt.kcg

# Rule to build all files generated by this target.
apps/imperio_tests/testInterrupt/CMakeFiles/testInterrupt.kcg.dir/build: testInterrupt.kcg

.PHONY : apps/imperio_tests/testInterrupt/CMakeFiles/testInterrupt.kcg.dir/build

apps/imperio_tests/testInterrupt/CMakeFiles/testInterrupt.kcg.dir/clean:
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/imperio_tests/testInterrupt && $(CMAKE_COMMAND) -P CMakeFiles/testInterrupt.kcg.dir/cmake_clean.cmake
.PHONY : apps/imperio_tests/testInterrupt/CMakeFiles/testInterrupt.kcg.dir/clean

apps/imperio_tests/testInterrupt/CMakeFiles/testInterrupt.kcg.dir/depend:
	cd /home/user/Desktop/Thesis/aftab/sw/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/Desktop/Thesis/aftab/sw /home/user/Desktop/Thesis/aftab/sw/apps/imperio_tests/testInterrupt /home/user/Desktop/Thesis/aftab/sw/build /home/user/Desktop/Thesis/aftab/sw/build/apps/imperio_tests/testInterrupt /home/user/Desktop/Thesis/aftab/sw/build/apps/imperio_tests/testInterrupt/CMakeFiles/testInterrupt.kcg.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : apps/imperio_tests/testInterrupt/CMakeFiles/testInterrupt.kcg.dir/depend

