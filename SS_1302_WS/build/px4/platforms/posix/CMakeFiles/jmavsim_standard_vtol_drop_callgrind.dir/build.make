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

# Utility rule file for jmavsim_standard_vtol_drop_callgrind.

# Include the progress variables for this target.
include platforms/posix/CMakeFiles/jmavsim_standard_vtol_drop_callgrind.dir/progress.make

platforms/posix/CMakeFiles/jmavsim_standard_vtol_drop_callgrind:
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/tmp && /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot/Tools/sitl_run.sh /home/ch13f_1419/E-Yantra/SS_1302_WS/devel/.private/px4/lib/px4/px4 callgrind jmavsim standard_vtol_drop none /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4

jmavsim_standard_vtol_drop_callgrind: platforms/posix/CMakeFiles/jmavsim_standard_vtol_drop_callgrind
jmavsim_standard_vtol_drop_callgrind: platforms/posix/CMakeFiles/jmavsim_standard_vtol_drop_callgrind.dir/build.make

.PHONY : jmavsim_standard_vtol_drop_callgrind

# Rule to build all files generated by this target.
platforms/posix/CMakeFiles/jmavsim_standard_vtol_drop_callgrind.dir/build: jmavsim_standard_vtol_drop_callgrind

.PHONY : platforms/posix/CMakeFiles/jmavsim_standard_vtol_drop_callgrind.dir/build

platforms/posix/CMakeFiles/jmavsim_standard_vtol_drop_callgrind.dir/clean:
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/platforms/posix && $(CMAKE_COMMAND) -P CMakeFiles/jmavsim_standard_vtol_drop_callgrind.dir/cmake_clean.cmake
.PHONY : platforms/posix/CMakeFiles/jmavsim_standard_vtol_drop_callgrind.dir/clean

platforms/posix/CMakeFiles/jmavsim_standard_vtol_drop_callgrind.dir/depend:
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot/platforms/posix /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4 /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/platforms/posix /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/platforms/posix/CMakeFiles/jmavsim_standard_vtol_drop_callgrind.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : platforms/posix/CMakeFiles/jmavsim_standard_vtol_drop_callgrind.dir/depend

