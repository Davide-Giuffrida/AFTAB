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

# Utility rule file for pooling.stim.txt.

# Include the progress variables for this target.
include apps/sequential_tests/pooling/CMakeFiles/pooling.stim.txt.dir/progress.make

apps/sequential_tests/pooling/CMakeFiles/pooling.stim.txt: apps/sequential_tests/pooling/vectors/stim.txt


apps/sequential_tests/pooling/vectors/stim.txt: apps/sequential_tests/pooling/pooling.s19
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/user/Desktop/Thesis/aftab/sw/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating vectors/stim.txt"

apps/sequential_tests/pooling/pooling.s19: apps/sequential_tests/pooling/pooling.elf
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/user/Desktop/Thesis/aftab/sw/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating pooling.s19"
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/sequential_tests/pooling && /opt/riscv/bin/riscv32-unknown-elf-objcopy --srec-len 1 --output-target=srec /home/user/Desktop/Thesis/aftab/sw/build/apps/sequential_tests/pooling/pooling.elf pooling.s19

pooling.stim.txt: apps/sequential_tests/pooling/CMakeFiles/pooling.stim.txt
pooling.stim.txt: apps/sequential_tests/pooling/vectors/stim.txt
pooling.stim.txt: apps/sequential_tests/pooling/pooling.s19
pooling.stim.txt: apps/sequential_tests/pooling/CMakeFiles/pooling.stim.txt.dir/build.make

.PHONY : pooling.stim.txt

# Rule to build all files generated by this target.
apps/sequential_tests/pooling/CMakeFiles/pooling.stim.txt.dir/build: pooling.stim.txt

.PHONY : apps/sequential_tests/pooling/CMakeFiles/pooling.stim.txt.dir/build

apps/sequential_tests/pooling/CMakeFiles/pooling.stim.txt.dir/clean:
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/sequential_tests/pooling && $(CMAKE_COMMAND) -P CMakeFiles/pooling.stim.txt.dir/cmake_clean.cmake
.PHONY : apps/sequential_tests/pooling/CMakeFiles/pooling.stim.txt.dir/clean

apps/sequential_tests/pooling/CMakeFiles/pooling.stim.txt.dir/depend:
	cd /home/user/Desktop/Thesis/aftab/sw/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/Desktop/Thesis/aftab/sw /home/user/Desktop/Thesis/aftab/sw/apps/sequential_tests/pooling /home/user/Desktop/Thesis/aftab/sw/build /home/user/Desktop/Thesis/aftab/sw/build/apps/sequential_tests/pooling /home/user/Desktop/Thesis/aftab/sw/build/apps/sequential_tests/pooling/CMakeFiles/pooling.stim.txt.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : apps/sequential_tests/pooling/CMakeFiles/pooling.stim.txt.dir/depend

