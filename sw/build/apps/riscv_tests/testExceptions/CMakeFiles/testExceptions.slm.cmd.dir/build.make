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

# Utility rule file for testExceptions.slm.cmd.

# Include the progress variables for this target.
include apps/riscv_tests/testExceptions/CMakeFiles/testExceptions.slm.cmd.dir/progress.make

apps/riscv_tests/testExceptions/CMakeFiles/testExceptions.slm.cmd: apps/riscv_tests/testExceptions/slm_files/l2_ram.slm


apps/riscv_tests/testExceptions/slm_files/l2_ram.slm: apps/riscv_tests/testExceptions/testExceptions.s19
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/user/Desktop/Thesis/aftab/sw/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating slm_files/l2_ram.slm"
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/riscv_tests/testExceptions/slm_files && /home/user/Desktop/Thesis/aftab/sw/utils/s19toslm.py ../testExceptions.s19
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/riscv_tests/testExceptions/slm_files && /usr/bin/cmake -E touch l2_ram.slm

apps/riscv_tests/testExceptions/testExceptions.s19: apps/riscv_tests/testExceptions/testExceptions.elf
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/user/Desktop/Thesis/aftab/sw/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating testExceptions.s19"
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/riscv_tests/testExceptions && /opt/riscv/bin/riscv32-unknown-elf-objcopy --srec-len 1 --output-target=srec /home/user/Desktop/Thesis/aftab/sw/build/apps/riscv_tests/testExceptions/testExceptions.elf testExceptions.s19

testExceptions.slm.cmd: apps/riscv_tests/testExceptions/CMakeFiles/testExceptions.slm.cmd
testExceptions.slm.cmd: apps/riscv_tests/testExceptions/slm_files/l2_ram.slm
testExceptions.slm.cmd: apps/riscv_tests/testExceptions/testExceptions.s19
testExceptions.slm.cmd: apps/riscv_tests/testExceptions/CMakeFiles/testExceptions.slm.cmd.dir/build.make

.PHONY : testExceptions.slm.cmd

# Rule to build all files generated by this target.
apps/riscv_tests/testExceptions/CMakeFiles/testExceptions.slm.cmd.dir/build: testExceptions.slm.cmd

.PHONY : apps/riscv_tests/testExceptions/CMakeFiles/testExceptions.slm.cmd.dir/build

apps/riscv_tests/testExceptions/CMakeFiles/testExceptions.slm.cmd.dir/clean:
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/riscv_tests/testExceptions && $(CMAKE_COMMAND) -P CMakeFiles/testExceptions.slm.cmd.dir/cmake_clean.cmake
.PHONY : apps/riscv_tests/testExceptions/CMakeFiles/testExceptions.slm.cmd.dir/clean

apps/riscv_tests/testExceptions/CMakeFiles/testExceptions.slm.cmd.dir/depend:
	cd /home/user/Desktop/Thesis/aftab/sw/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/Desktop/Thesis/aftab/sw /home/user/Desktop/Thesis/aftab/sw/apps/riscv_tests/testExceptions /home/user/Desktop/Thesis/aftab/sw/build /home/user/Desktop/Thesis/aftab/sw/build/apps/riscv_tests/testExceptions /home/user/Desktop/Thesis/aftab/sw/build/apps/riscv_tests/testExceptions/CMakeFiles/testExceptions.slm.cmd.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : apps/riscv_tests/testExceptions/CMakeFiles/testExceptions.slm.cmd.dir/depend

