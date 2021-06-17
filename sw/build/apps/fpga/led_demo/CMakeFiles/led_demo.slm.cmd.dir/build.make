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

# Utility rule file for led_demo.slm.cmd.

# Include the progress variables for this target.
include apps/fpga/led_demo/CMakeFiles/led_demo.slm.cmd.dir/progress.make

apps/fpga/led_demo/CMakeFiles/led_demo.slm.cmd: apps/fpga/led_demo/slm_files/l2_ram.slm


apps/fpga/led_demo/slm_files/l2_ram.slm: apps/fpga/led_demo/led_demo.s19
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/user/Desktop/Thesis/aftab/sw/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating slm_files/l2_ram.slm"
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/fpga/led_demo/slm_files && /home/user/Desktop/Thesis/aftab/sw/utils/s19toslm.py ../led_demo.s19
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/fpga/led_demo/slm_files && /usr/bin/cmake -E touch l2_ram.slm

apps/fpga/led_demo/led_demo.s19: apps/fpga/led_demo/led_demo.elf
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/user/Desktop/Thesis/aftab/sw/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating led_demo.s19"
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/fpga/led_demo && /opt/riscv/bin/riscv32-unknown-elf-objcopy --srec-len 1 --output-target=srec /home/user/Desktop/Thesis/aftab/sw/build/apps/fpga/led_demo/led_demo.elf led_demo.s19

led_demo.slm.cmd: apps/fpga/led_demo/CMakeFiles/led_demo.slm.cmd
led_demo.slm.cmd: apps/fpga/led_demo/slm_files/l2_ram.slm
led_demo.slm.cmd: apps/fpga/led_demo/led_demo.s19
led_demo.slm.cmd: apps/fpga/led_demo/CMakeFiles/led_demo.slm.cmd.dir/build.make

.PHONY : led_demo.slm.cmd

# Rule to build all files generated by this target.
apps/fpga/led_demo/CMakeFiles/led_demo.slm.cmd.dir/build: led_demo.slm.cmd

.PHONY : apps/fpga/led_demo/CMakeFiles/led_demo.slm.cmd.dir/build

apps/fpga/led_demo/CMakeFiles/led_demo.slm.cmd.dir/clean:
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/fpga/led_demo && $(CMAKE_COMMAND) -P CMakeFiles/led_demo.slm.cmd.dir/cmake_clean.cmake
.PHONY : apps/fpga/led_demo/CMakeFiles/led_demo.slm.cmd.dir/clean

apps/fpga/led_demo/CMakeFiles/led_demo.slm.cmd.dir/depend:
	cd /home/user/Desktop/Thesis/aftab/sw/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/Desktop/Thesis/aftab/sw /home/user/Desktop/Thesis/aftab/sw/apps/fpga/led_demo /home/user/Desktop/Thesis/aftab/sw/build /home/user/Desktop/Thesis/aftab/sw/build/apps/fpga/led_demo /home/user/Desktop/Thesis/aftab/sw/build/apps/fpga/led_demo/CMakeFiles/led_demo.slm.cmd.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : apps/fpga/led_demo/CMakeFiles/led_demo.slm.cmd.dir/depend

