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
include apps/riscv_tests/testCSR/CMakeFiles/testCSR.elf.dir/depend.make

# Include the progress variables for this target.
include apps/riscv_tests/testCSR/CMakeFiles/testCSR.elf.dir/progress.make

# Include the compile flags for this target's objects.
include apps/riscv_tests/testCSR/CMakeFiles/testCSR.elf.dir/flags.make

apps/riscv_tests/testCSR/CMakeFiles/testCSR.elf.dir/testCSR.c.o: apps/riscv_tests/testCSR/CMakeFiles/testCSR.elf.dir/flags.make
apps/riscv_tests/testCSR/CMakeFiles/testCSR.elf.dir/testCSR.c.o: ../apps/riscv_tests/testCSR/testCSR.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/Desktop/Thesis/aftab/sw/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object apps/riscv_tests/testCSR/CMakeFiles/testCSR.elf.dir/testCSR.c.o"
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/riscv_tests/testCSR && /opt/riscv/bin/riscv32-unknown-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/testCSR.elf.dir/testCSR.c.o   -c /home/user/Desktop/Thesis/aftab/sw/apps/riscv_tests/testCSR/testCSR.c

apps/riscv_tests/testCSR/CMakeFiles/testCSR.elf.dir/testCSR.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/testCSR.elf.dir/testCSR.c.i"
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/riscv_tests/testCSR && /opt/riscv/bin/riscv32-unknown-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/user/Desktop/Thesis/aftab/sw/apps/riscv_tests/testCSR/testCSR.c > CMakeFiles/testCSR.elf.dir/testCSR.c.i

apps/riscv_tests/testCSR/CMakeFiles/testCSR.elf.dir/testCSR.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/testCSR.elf.dir/testCSR.c.s"
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/riscv_tests/testCSR && /opt/riscv/bin/riscv32-unknown-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/user/Desktop/Thesis/aftab/sw/apps/riscv_tests/testCSR/testCSR.c -o CMakeFiles/testCSR.elf.dir/testCSR.c.s

# Object files for target testCSR.elf
testCSR_elf_OBJECTS = \
"CMakeFiles/testCSR.elf.dir/testCSR.c.o"

# External object files for target testCSR.elf
testCSR_elf_EXTERNAL_OBJECTS = \
"/home/user/Desktop/Thesis/aftab/sw/build/CMakeFiles/crt0.dir/ref/crt0.riscv.S.o"

apps/riscv_tests/testCSR/testCSR.elf: apps/riscv_tests/testCSR/CMakeFiles/testCSR.elf.dir/testCSR.c.o
apps/riscv_tests/testCSR/testCSR.elf: CMakeFiles/crt0.dir/ref/crt0.riscv.S.o
apps/riscv_tests/testCSR/testCSR.elf: apps/riscv_tests/testCSR/CMakeFiles/testCSR.elf.dir/build.make
apps/riscv_tests/testCSR/testCSR.elf: libs/bench_lib/libbench.a
apps/riscv_tests/testCSR/testCSR.elf: libs/string_lib/libstring.a
apps/riscv_tests/testCSR/testCSR.elf: libs/sys_lib/libsys.a
apps/riscv_tests/testCSR/testCSR.elf: apps/riscv_tests/testCSR/CMakeFiles/testCSR.elf.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/user/Desktop/Thesis/aftab/sw/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C executable testCSR.elf"
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/riscv_tests/testCSR && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/testCSR.elf.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
apps/riscv_tests/testCSR/CMakeFiles/testCSR.elf.dir/build: apps/riscv_tests/testCSR/testCSR.elf

.PHONY : apps/riscv_tests/testCSR/CMakeFiles/testCSR.elf.dir/build

apps/riscv_tests/testCSR/CMakeFiles/testCSR.elf.dir/clean:
	cd /home/user/Desktop/Thesis/aftab/sw/build/apps/riscv_tests/testCSR && $(CMAKE_COMMAND) -P CMakeFiles/testCSR.elf.dir/cmake_clean.cmake
.PHONY : apps/riscv_tests/testCSR/CMakeFiles/testCSR.elf.dir/clean

apps/riscv_tests/testCSR/CMakeFiles/testCSR.elf.dir/depend:
	cd /home/user/Desktop/Thesis/aftab/sw/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/Desktop/Thesis/aftab/sw /home/user/Desktop/Thesis/aftab/sw/apps/riscv_tests/testCSR /home/user/Desktop/Thesis/aftab/sw/build /home/user/Desktop/Thesis/aftab/sw/build/apps/riscv_tests/testCSR /home/user/Desktop/Thesis/aftab/sw/build/apps/riscv_tests/testCSR/CMakeFiles/testCSR.elf.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : apps/riscv_tests/testCSR/CMakeFiles/testCSR.elf.dir/depend

