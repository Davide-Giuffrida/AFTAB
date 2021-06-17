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
CMAKE_SOURCE_DIR = /home/user/Desktop/Thesis/mc2101/sw

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/user/Desktop/Thesis/mc2101/sw/build

# Utility rule file for boot_code.stim.txt.

# Include the progress variables for this target.
include apps/boot_code/CMakeFiles/boot_code.stim.txt.dir/progress.make

apps/boot_code/CMakeFiles/boot_code.stim.txt: apps/boot_code/vectors/stim.txt


apps/boot_code/vectors/stim.txt: apps/boot_code/boot_code.s19
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/user/Desktop/Thesis/mc2101/sw/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating vectors/stim.txt"

apps/boot_code/boot_code.s19: apps/boot_code/boot_code.elf
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/user/Desktop/Thesis/mc2101/sw/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating boot_code.s19"
	cd /home/user/Desktop/Thesis/mc2101/sw/build/apps/boot_code && /opt/riscv/bin/riscv32-unknown-elf-objcopy --srec-len 1 --output-target=srec /home/user/Desktop/Thesis/mc2101/sw/build/apps/boot_code/boot_code.elf boot_code.s19

boot_code.stim.txt: apps/boot_code/CMakeFiles/boot_code.stim.txt
boot_code.stim.txt: apps/boot_code/vectors/stim.txt
boot_code.stim.txt: apps/boot_code/boot_code.s19
boot_code.stim.txt: apps/boot_code/CMakeFiles/boot_code.stim.txt.dir/build.make

.PHONY : boot_code.stim.txt

# Rule to build all files generated by this target.
apps/boot_code/CMakeFiles/boot_code.stim.txt.dir/build: boot_code.stim.txt

.PHONY : apps/boot_code/CMakeFiles/boot_code.stim.txt.dir/build

apps/boot_code/CMakeFiles/boot_code.stim.txt.dir/clean:
	cd /home/user/Desktop/Thesis/mc2101/sw/build/apps/boot_code && $(CMAKE_COMMAND) -P CMakeFiles/boot_code.stim.txt.dir/cmake_clean.cmake
.PHONY : apps/boot_code/CMakeFiles/boot_code.stim.txt.dir/clean

apps/boot_code/CMakeFiles/boot_code.stim.txt.dir/depend:
	cd /home/user/Desktop/Thesis/mc2101/sw/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/Desktop/Thesis/mc2101/sw /home/user/Desktop/Thesis/mc2101/sw/apps/boot_code /home/user/Desktop/Thesis/mc2101/sw/build /home/user/Desktop/Thesis/mc2101/sw/build/apps/boot_code /home/user/Desktop/Thesis/mc2101/sw/build/apps/boot_code/CMakeFiles/boot_code.stim.txt.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : apps/boot_code/CMakeFiles/boot_code.stim.txt.dir/depend

