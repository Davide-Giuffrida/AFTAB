# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /media/sf_Shared/pulpino/sw

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /media/sf_Shared/pulpino/sw/build

# Utility rule file for testMUL.slm.cmd.

# Include the progress variables for this target.
include apps/riscv_tests/testMUL/CMakeFiles/testMUL.slm.cmd.dir/progress.make

apps/riscv_tests/testMUL/CMakeFiles/testMUL.slm.cmd: apps/riscv_tests/testMUL/slm_files/l2_ram.slm


apps/riscv_tests/testMUL/slm_files/l2_ram.slm: apps/riscv_tests/testMUL/testMUL.s19
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/media/sf_Shared/pulpino/sw/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating slm_files/l2_ram.slm"
	cd /media/sf_Shared/pulpino/sw/build/apps/riscv_tests/testMUL/slm_files && /media/sf_Shared/pulpino/sw/utils/s19toslm.py ../testMUL.s19
	cd /media/sf_Shared/pulpino/sw/build/apps/riscv_tests/testMUL/slm_files && /usr/bin/cmake -E touch l2_ram.slm

apps/riscv_tests/testMUL/testMUL.s19: apps/riscv_tests/testMUL/testMUL.elf
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/media/sf_Shared/pulpino/sw/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating testMUL.s19"
	cd /media/sf_Shared/pulpino/sw/build/apps/riscv_tests/testMUL && /opt/riscv/bin/riscv32-unknown-elf-objcopy --srec-len 1 --output-target=srec /media/sf_Shared/pulpino/sw/build/apps/riscv_tests/testMUL/testMUL.elf testMUL.s19

testMUL.slm.cmd: apps/riscv_tests/testMUL/CMakeFiles/testMUL.slm.cmd
testMUL.slm.cmd: apps/riscv_tests/testMUL/slm_files/l2_ram.slm
testMUL.slm.cmd: apps/riscv_tests/testMUL/testMUL.s19
testMUL.slm.cmd: apps/riscv_tests/testMUL/CMakeFiles/testMUL.slm.cmd.dir/build.make

.PHONY : testMUL.slm.cmd

# Rule to build all files generated by this target.
apps/riscv_tests/testMUL/CMakeFiles/testMUL.slm.cmd.dir/build: testMUL.slm.cmd

.PHONY : apps/riscv_tests/testMUL/CMakeFiles/testMUL.slm.cmd.dir/build

apps/riscv_tests/testMUL/CMakeFiles/testMUL.slm.cmd.dir/clean:
	cd /media/sf_Shared/pulpino/sw/build/apps/riscv_tests/testMUL && $(CMAKE_COMMAND) -P CMakeFiles/testMUL.slm.cmd.dir/cmake_clean.cmake
.PHONY : apps/riscv_tests/testMUL/CMakeFiles/testMUL.slm.cmd.dir/clean

apps/riscv_tests/testMUL/CMakeFiles/testMUL.slm.cmd.dir/depend:
	cd /media/sf_Shared/pulpino/sw/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /media/sf_Shared/pulpino/sw /media/sf_Shared/pulpino/sw/apps/riscv_tests/testMUL /media/sf_Shared/pulpino/sw/build /media/sf_Shared/pulpino/sw/build/apps/riscv_tests/testMUL /media/sf_Shared/pulpino/sw/build/apps/riscv_tests/testMUL/CMakeFiles/testMUL.slm.cmd.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : apps/riscv_tests/testMUL/CMakeFiles/testMUL.slm.cmd.dir/depend

