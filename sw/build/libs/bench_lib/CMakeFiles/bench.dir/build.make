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

# Include any dependencies generated for this target.
include libs/bench_lib/CMakeFiles/bench.dir/depend.make

# Include the progress variables for this target.
include libs/bench_lib/CMakeFiles/bench.dir/progress.make

# Include the compile flags for this target's objects.
include libs/bench_lib/CMakeFiles/bench.dir/flags.make

libs/bench_lib/CMakeFiles/bench.dir/src/bench.c.o: libs/bench_lib/CMakeFiles/bench.dir/flags.make
libs/bench_lib/CMakeFiles/bench.dir/src/bench.c.o: ../libs/bench_lib/src/bench.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/Desktop/Thesis/aftab/sw/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object libs/bench_lib/CMakeFiles/bench.dir/src/bench.c.o"
	cd /home/user/Desktop/Thesis/aftab/sw/build/libs/bench_lib && /opt/riscv/bin/riscv32-unknown-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/bench.dir/src/bench.c.o   -c /home/user/Desktop/Thesis/aftab/sw/libs/bench_lib/src/bench.c

libs/bench_lib/CMakeFiles/bench.dir/src/bench.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/bench.dir/src/bench.c.i"
	cd /home/user/Desktop/Thesis/aftab/sw/build/libs/bench_lib && /opt/riscv/bin/riscv32-unknown-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/user/Desktop/Thesis/aftab/sw/libs/bench_lib/src/bench.c > CMakeFiles/bench.dir/src/bench.c.i

libs/bench_lib/CMakeFiles/bench.dir/src/bench.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/bench.dir/src/bench.c.s"
	cd /home/user/Desktop/Thesis/aftab/sw/build/libs/bench_lib && /opt/riscv/bin/riscv32-unknown-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/user/Desktop/Thesis/aftab/sw/libs/bench_lib/src/bench.c -o CMakeFiles/bench.dir/src/bench.c.s

# Object files for target bench
bench_OBJECTS = \
"CMakeFiles/bench.dir/src/bench.c.o"

# External object files for target bench
bench_EXTERNAL_OBJECTS =

libs/bench_lib/libbench.a: libs/bench_lib/CMakeFiles/bench.dir/src/bench.c.o
libs/bench_lib/libbench.a: libs/bench_lib/CMakeFiles/bench.dir/build.make
libs/bench_lib/libbench.a: libs/bench_lib/CMakeFiles/bench.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/user/Desktop/Thesis/aftab/sw/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C static library libbench.a"
	cd /home/user/Desktop/Thesis/aftab/sw/build/libs/bench_lib && $(CMAKE_COMMAND) -P CMakeFiles/bench.dir/cmake_clean_target.cmake
	cd /home/user/Desktop/Thesis/aftab/sw/build/libs/bench_lib && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/bench.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
libs/bench_lib/CMakeFiles/bench.dir/build: libs/bench_lib/libbench.a

.PHONY : libs/bench_lib/CMakeFiles/bench.dir/build

libs/bench_lib/CMakeFiles/bench.dir/clean:
	cd /home/user/Desktop/Thesis/aftab/sw/build/libs/bench_lib && $(CMAKE_COMMAND) -P CMakeFiles/bench.dir/cmake_clean.cmake
.PHONY : libs/bench_lib/CMakeFiles/bench.dir/clean

libs/bench_lib/CMakeFiles/bench.dir/depend:
	cd /home/user/Desktop/Thesis/aftab/sw/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/Desktop/Thesis/aftab/sw /home/user/Desktop/Thesis/aftab/sw/libs/bench_lib /home/user/Desktop/Thesis/aftab/sw/build /home/user/Desktop/Thesis/aftab/sw/build/libs/bench_lib /home/user/Desktop/Thesis/aftab/sw/build/libs/bench_lib/CMakeFiles/bench.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : libs/bench_lib/CMakeFiles/bench.dir/depend

