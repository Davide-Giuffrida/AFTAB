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

# Utility rule file for perfbench.ipm.sc.ps.

# Include the progress variables for this target.
include apps/bench/ipm/CMakeFiles/perfbench.ipm.sc.ps.dir/progress.make

apps/bench/ipm/CMakeFiles/perfbench.ipm.sc.ps:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/user/Desktop/Thesis/aftab/sw/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Running scan chain testing (post synthesis)"
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/bench/ipm && tcsh -c env\ VSIM_DIR=/home/user/Desktop/Thesis/aftab/vsim\ USE_ZERO_RISCY=1\ RISCY_RV32F=0\ ZERO_RV32M=1\ ZERO_RV32E=0\ PL_NETLIST=\ TB_TEST=""\ /usr/local/modelsim/modelsim_ase/bin/vsim\ \ -64\ -do\ 'source\ tcl_files/run_sc_ps.tcl\;'

perfbench.ipm.sc.ps: apps/bench/ipm/CMakeFiles/perfbench.ipm.sc.ps
perfbench.ipm.sc.ps: apps/bench/ipm/CMakeFiles/perfbench.ipm.sc.ps.dir/build.make

.PHONY : perfbench.ipm.sc.ps

# Rule to build all files generated by this target.
apps/bench/ipm/CMakeFiles/perfbench.ipm.sc.ps.dir/build: perfbench.ipm.sc.ps

.PHONY : apps/bench/ipm/CMakeFiles/perfbench.ipm.sc.ps.dir/build

apps/bench/ipm/CMakeFiles/perfbench.ipm.sc.ps.dir/clean:
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/bench/ipm && $(CMAKE_COMMAND) -P CMakeFiles/perfbench.ipm.sc.ps.dir/cmake_clean.cmake
.PHONY : apps/bench/ipm/CMakeFiles/perfbench.ipm.sc.ps.dir/clean

apps/bench/ipm/CMakeFiles/perfbench.ipm.sc.ps.dir/depend:
	cd /home/user/Desktop/Thesis/aftab/sw/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/Desktop/Thesis/aftab/sw /home/user/Desktop/Thesis/aftab/sw/apps/bench/ipm /home/user/Desktop/Thesis/aftab/sw/build /home/user/Desktop/Thesis/aftab/sw/build/apps/bench/ipm /home/user/Desktop/Thesis/aftab/sw/build/apps/bench/ipm/CMakeFiles/perfbench.ipm.sc.ps.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : apps/bench/ipm/CMakeFiles/perfbench.ipm.sc.ps.dir/depend

