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

# Utility rule file for freertos.links.

# Include the progress variables for this target.
include apps/freertos/CMakeFiles/freertos.links.dir/progress.make

apps/freertos/CMakeFiles/freertos.links: apps/freertos/modelsim.ini
apps/freertos/CMakeFiles/freertos.links: apps/freertos/work
apps/freertos/CMakeFiles/freertos.links: apps/freertos/tcl_files
apps/freertos/CMakeFiles/freertos.links: apps/freertos/waves


apps/freertos/modelsim.ini:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/user/Desktop/Thesis/aftab/sw/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating modelsim.ini"
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/freertos && /usr/bin/cmake -E create_symlink /home/user/Desktop/Thesis/aftab/vsim/modelsim.ini /home/user/Desktop/Thesis/aftab/sw/build/apps/freertos//modelsim.ini

apps/freertos/work:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/user/Desktop/Thesis/aftab/sw/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating work"
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/freertos && /usr/bin/cmake -E create_symlink /home/user/Desktop/Thesis/aftab/vsim/work /home/user/Desktop/Thesis/aftab/sw/build/apps/freertos//work

apps/freertos/tcl_files:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/user/Desktop/Thesis/aftab/sw/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Generating tcl_files"
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/freertos && /usr/bin/cmake -E create_symlink /home/user/Desktop/Thesis/aftab/vsim/tcl_files /home/user/Desktop/Thesis/aftab/sw/build/apps/freertos//tcl_files

apps/freertos/waves:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/user/Desktop/Thesis/aftab/sw/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Generating waves"
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/freertos && /usr/bin/cmake -E create_symlink /home/user/Desktop/Thesis/aftab/vsim/waves /home/user/Desktop/Thesis/aftab/sw/build/apps/freertos//waves

freertos.links: apps/freertos/CMakeFiles/freertos.links
freertos.links: apps/freertos/modelsim.ini
freertos.links: apps/freertos/work
freertos.links: apps/freertos/tcl_files
freertos.links: apps/freertos/waves
freertos.links: apps/freertos/CMakeFiles/freertos.links.dir/build.make

.PHONY : freertos.links

# Rule to build all files generated by this target.
apps/freertos/CMakeFiles/freertos.links.dir/build: freertos.links

.PHONY : apps/freertos/CMakeFiles/freertos.links.dir/build

apps/freertos/CMakeFiles/freertos.links.dir/clean:
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/freertos && $(CMAKE_COMMAND) -P CMakeFiles/freertos.links.dir/cmake_clean.cmake
.PHONY : apps/freertos/CMakeFiles/freertos.links.dir/clean

apps/freertos/CMakeFiles/freertos.links.dir/depend:
	cd /home/user/Desktop/Thesis/aftab/sw/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/Desktop/Thesis/aftab/sw /home/user/Desktop/Thesis/aftab/sw/apps/freertos /home/user/Desktop/Thesis/aftab/sw/build /home/user/Desktop/Thesis/aftab/sw/build/apps/freertos /home/user/Desktop/Thesis/aftab/sw/build/apps/freertos/CMakeFiles/freertos.links.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : apps/freertos/CMakeFiles/freertos.links.dir/depend

