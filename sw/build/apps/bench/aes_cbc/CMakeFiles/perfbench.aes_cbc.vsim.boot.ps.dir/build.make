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

# Utility rule file for perfbench.aes_cbc.vsim.boot.ps.

# Include the progress variables for this target.
include apps/bench/aes_cbc/CMakeFiles/perfbench.aes_cbc.vsim.boot.ps.dir/progress.make

apps/bench/aes_cbc/CMakeFiles/perfbench.aes_cbc.vsim.boot.ps:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/user/Desktop/Thesis/aftab/sw/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Running perfbench.aes_cbc in ModelSim (post synthesis)"
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/bench/aes_cbc && /usr/bin/cmake -E remove stdout/*
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/bench/aes_cbc && /usr/bin/cmake -E remove FS/*
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/bench/aes_cbc && tcsh -c env\ VSIM_DIR=/home/user/Desktop/Thesis/aftab/vsim\ USE_ZERO_RISCY=1\ RISCY_RV32F=0\ ZERO_RV32M=1\ ZERO_RV32E=0\ PL_NETLIST=\ TB_TEST=""\ /usr/local/modelsim/modelsim_ase/bin/vsim\ \ -64\ -do\ 'source\ tcl_files/run_boot_ps.tcl\;'

perfbench.aes_cbc.vsim.boot.ps: apps/bench/aes_cbc/CMakeFiles/perfbench.aes_cbc.vsim.boot.ps
perfbench.aes_cbc.vsim.boot.ps: apps/bench/aes_cbc/CMakeFiles/perfbench.aes_cbc.vsim.boot.ps.dir/build.make

.PHONY : perfbench.aes_cbc.vsim.boot.ps

# Rule to build all files generated by this target.
apps/bench/aes_cbc/CMakeFiles/perfbench.aes_cbc.vsim.boot.ps.dir/build: perfbench.aes_cbc.vsim.boot.ps

.PHONY : apps/bench/aes_cbc/CMakeFiles/perfbench.aes_cbc.vsim.boot.ps.dir/build

apps/bench/aes_cbc/CMakeFiles/perfbench.aes_cbc.vsim.boot.ps.dir/clean:
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/bench/aes_cbc && $(CMAKE_COMMAND) -P CMakeFiles/perfbench.aes_cbc.vsim.boot.ps.dir/cmake_clean.cmake
.PHONY : apps/bench/aes_cbc/CMakeFiles/perfbench.aes_cbc.vsim.boot.ps.dir/clean

apps/bench/aes_cbc/CMakeFiles/perfbench.aes_cbc.vsim.boot.ps.dir/depend:
	cd /home/user/Desktop/Thesis/aftab/sw/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/Desktop/Thesis/aftab/sw /home/user/Desktop/Thesis/aftab/sw/apps/bench/aes_cbc /home/user/Desktop/Thesis/aftab/sw/build /home/user/Desktop/Thesis/aftab/sw/build/apps/bench/aes_cbc /home/user/Desktop/Thesis/aftab/sw/build/apps/bench/aes_cbc/CMakeFiles/perfbench.aes_cbc.vsim.boot.ps.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : apps/bench/aes_cbc/CMakeFiles/perfbench.aes_cbc.vsim.boot.ps.dir/depend

