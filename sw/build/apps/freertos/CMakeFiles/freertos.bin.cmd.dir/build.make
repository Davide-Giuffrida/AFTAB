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

# Utility rule file for freertos.bin.cmd.

# Include the progress variables for this target.
include apps/freertos/CMakeFiles/freertos.bin.cmd.dir/progress.make

apps/freertos/CMakeFiles/freertos.bin.cmd: apps/freertos/freertos.bin


apps/freertos/freertos.bin: apps/freertos/freertos.elf
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/user/Desktop/Thesis/aftab/sw/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating freertos.bin"
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/freertos && /opt/riscv/bin/riscv32-unknown-elf-objcopy -R .debug_frame -R .comment -R .stack -R .heapsram -R .heapscm -R .scmlock -O binary /home/user/Desktop/Thesis/aftab/sw/build/apps/freertos/freertos.elf freertos.bin

freertos.bin.cmd: apps/freertos/CMakeFiles/freertos.bin.cmd
freertos.bin.cmd: apps/freertos/freertos.bin
freertos.bin.cmd: apps/freertos/CMakeFiles/freertos.bin.cmd.dir/build.make

.PHONY : freertos.bin.cmd

# Rule to build all files generated by this target.
apps/freertos/CMakeFiles/freertos.bin.cmd.dir/build: freertos.bin.cmd

.PHONY : apps/freertos/CMakeFiles/freertos.bin.cmd.dir/build

apps/freertos/CMakeFiles/freertos.bin.cmd.dir/clean:
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/freertos && $(CMAKE_COMMAND) -P CMakeFiles/freertos.bin.cmd.dir/cmake_clean.cmake
.PHONY : apps/freertos/CMakeFiles/freertos.bin.cmd.dir/clean

apps/freertos/CMakeFiles/freertos.bin.cmd.dir/depend:
	cd /home/user/Desktop/Thesis/aftab/sw/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/Desktop/Thesis/aftab/sw /home/user/Desktop/Thesis/aftab/sw/apps/freertos /home/user/Desktop/Thesis/aftab/sw/build /home/user/Desktop/Thesis/aftab/sw/build/apps/freertos /home/user/Desktop/Thesis/aftab/sw/build/apps/freertos/CMakeFiles/freertos.bin.cmd.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : apps/freertos/CMakeFiles/freertos.bin.cmd.dir/depend

