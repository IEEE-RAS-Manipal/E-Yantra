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
CMAKE_SOURCE_DIR = /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4

# Include any dependencies generated for this target.
include src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/depend.make

# Include the progress variables for this target.
include src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/progress.make

# Include the compile flags for this target's objects.
include src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/flags.make

src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/sd_bench.c.o: src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/flags.make
src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/sd_bench.c.o: /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot/src/systemcmds/sd_bench/sd_bench.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/sd_bench.c.o"
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/src/systemcmds/sd_bench && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/systemcmds__sd_bench.dir/sd_bench.c.o   -c /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot/src/systemcmds/sd_bench/sd_bench.c

src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/sd_bench.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/systemcmds__sd_bench.dir/sd_bench.c.i"
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/src/systemcmds/sd_bench && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot/src/systemcmds/sd_bench/sd_bench.c > CMakeFiles/systemcmds__sd_bench.dir/sd_bench.c.i

src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/sd_bench.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/systemcmds__sd_bench.dir/sd_bench.c.s"
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/src/systemcmds/sd_bench && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot/src/systemcmds/sd_bench/sd_bench.c -o CMakeFiles/systemcmds__sd_bench.dir/sd_bench.c.s

# Object files for target systemcmds__sd_bench
systemcmds__sd_bench_OBJECTS = \
"CMakeFiles/systemcmds__sd_bench.dir/sd_bench.c.o"

# External object files for target systemcmds__sd_bench
systemcmds__sd_bench_EXTERNAL_OBJECTS =

/home/ch13f_1419/E-Yantra/SS_1302_WS/devel/.private/px4/lib/libsystemcmds__sd_bench.a: src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/sd_bench.c.o
/home/ch13f_1419/E-Yantra/SS_1302_WS/devel/.private/px4/lib/libsystemcmds__sd_bench.a: src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/build.make
/home/ch13f_1419/E-Yantra/SS_1302_WS/devel/.private/px4/lib/libsystemcmds__sd_bench.a: src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library /home/ch13f_1419/E-Yantra/SS_1302_WS/devel/.private/px4/lib/libsystemcmds__sd_bench.a"
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/src/systemcmds/sd_bench && $(CMAKE_COMMAND) -P CMakeFiles/systemcmds__sd_bench.dir/cmake_clean_target.cmake
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/src/systemcmds/sd_bench && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/systemcmds__sd_bench.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/build: /home/ch13f_1419/E-Yantra/SS_1302_WS/devel/.private/px4/lib/libsystemcmds__sd_bench.a

.PHONY : src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/build

src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/clean:
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/src/systemcmds/sd_bench && $(CMAKE_COMMAND) -P CMakeFiles/systemcmds__sd_bench.dir/cmake_clean.cmake
.PHONY : src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/clean

src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/depend:
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot/src/systemcmds/sd_bench /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4 /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/src/systemcmds/sd_bench /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/depend

