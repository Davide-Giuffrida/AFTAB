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

# Utility rule file for testUART.s19.cmd.

# Include the progress variables for this target.
include apps/imperio_tests/testUART/CMakeFiles/testUART.s19.cmd.dir/progress.make

apps/imperio_tests/testUART/CMakeFiles/testUART.s19.cmd: apps/imperio_tests/testUART/testUART.s19


apps/imperio_tests/testUART/testUART.s19: apps/imperio_tests/testUART/testUART.elf
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/user/Desktop/Thesis/aftab/sw/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating testUART.s19"
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/imperio_tests/testUART && /opt/riscv/bin/riscv32-unknown-elf-objcopy --srec-len 1 --output-target=srec /home/user/Desktop/Thesis/aftab/sw/build/apps/imperio_tests/testUART/testUART.elf testUART.s19

testUART.s19.cmd: apps/imperio_tests/testUART/CMakeFiles/testUART.s19.cmd
testUART.s19.cmd: apps/imperio_tests/testUART/testUART.s19
testUART.s19.cmd: apps/imperio_tests/testUART/CMakeFiles/testUART.s19.cmd.dir/build.make

.PHONY : testUART.s19.cmd

# Rule to build all files generated by this target.
apps/imperio_tests/testUART/CMakeFiles/testUART.s19.cmd.dir/build: testUART.s19.cmd

.PHONY : apps/imperio_tests/testUART/CMakeFiles/testUART.s19.cmd.dir/build

apps/imperio_tests/testUART/CMakeFiles/testUART.s19.cmd.dir/clean:
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/imperio_tests/testUART && $(CMAKE_COMMAND) -P CMakeFiles/testUART.s19.cmd.dir/cmake_clean.cmake
.PHONY : apps/imperio_tests/testUART/CMakeFiles/testUART.s19.cmd.dir/clean

apps/imperio_tests/testUART/CMakeFiles/testUART.s19.cmd.dir/depend:
	cd /home/user/Desktop/Thesis/aftab/sw/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/Desktop/Thesis/aftab/sw /home/user/Desktop/Thesis/aftab/sw/apps/imperio_tests/testUART /home/user/Desktop/Thesis/aftab/sw/build /home/user/Desktop/Thesis/aftab/sw/build/apps/imperio_tests/testUART /home/user/Desktop/Thesis/aftab/sw/build/apps/imperio_tests/testUART/CMakeFiles/testUART.s19.cmd.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : apps/imperio_tests/testUART/CMakeFiles/testUART.s19.cmd.dir/depend

