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

# Utility rule file for perfbench.fdctfst.read.

# Include the progress variables for this target.
include apps/bench/fdctfst/CMakeFiles/perfbench.fdctfst.read.dir/progress.make

perfbench.fdctfst.read: apps/bench/fdctfst/CMakeFiles/perfbench.fdctfst.read.dir/build.make
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/bench/fdctfst && /opt/riscv/bin/riscv32-unknown-elf-objdump -Mmarch=rv32im -d /home/user/Desktop/Thesis/aftab/sw/build/apps/bench/fdctfst/perfbench.fdctfst.elf > perfbench.fdctfst.read
.PHONY : perfbench.fdctfst.read

# Rule to build all files generated by this target.
apps/bench/fdctfst/CMakeFiles/perfbench.fdctfst.read.dir/build: perfbench.fdctfst.read

.PHONY : apps/bench/fdctfst/CMakeFiles/perfbench.fdctfst.read.dir/build

apps/bench/fdctfst/CMakeFiles/perfbench.fdctfst.read.dir/clean:
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/bench/fdctfst && $(CMAKE_COMMAND) -P CMakeFiles/perfbench.fdctfst.read.dir/cmake_clean.cmake
.PHONY : apps/bench/fdctfst/CMakeFiles/perfbench.fdctfst.read.dir/clean

apps/bench/fdctfst/CMakeFiles/perfbench.fdctfst.read.dir/depend:
	cd /home/user/Desktop/Thesis/aftab/sw/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/Desktop/Thesis/aftab/sw /home/user/Desktop/Thesis/aftab/sw/apps/bench/fdctfst /home/user/Desktop/Thesis/aftab/sw/build /home/user/Desktop/Thesis/aftab/sw/build/apps/bench/fdctfst /home/user/Desktop/Thesis/aftab/sw/build/apps/bench/fdctfst/CMakeFiles/perfbench.fdctfst.read.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : apps/bench/fdctfst/CMakeFiles/perfbench.fdctfst.read.dir/depend

