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

# Utility rule file for git_matrix.

# Include the progress variables for this target.
include src/lib/CMakeFiles/git_matrix.dir/progress.make

src/lib/CMakeFiles/git_matrix: src/lib/git_init_matrix.stamp


src/lib/git_init_matrix.stamp: /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot/.gitmodules
src/lib/git_init_matrix.stamp: /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot/src/lib/matrix/.git
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "git submodule src/lib/matrix"
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot && Tools/check_submodules.sh src/lib/matrix
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot && /usr/bin/cmake -E touch /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/src/lib/git_init_matrix.stamp

git_matrix: src/lib/CMakeFiles/git_matrix
git_matrix: src/lib/git_init_matrix.stamp
git_matrix: src/lib/CMakeFiles/git_matrix.dir/build.make

.PHONY : git_matrix

# Rule to build all files generated by this target.
src/lib/CMakeFiles/git_matrix.dir/build: git_matrix

.PHONY : src/lib/CMakeFiles/git_matrix.dir/build

src/lib/CMakeFiles/git_matrix.dir/clean:
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/src/lib && $(CMAKE_COMMAND) -P CMakeFiles/git_matrix.dir/cmake_clean.cmake
.PHONY : src/lib/CMakeFiles/git_matrix.dir/clean

src/lib/CMakeFiles/git_matrix.dir/depend:
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot/src/lib /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4 /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/src/lib /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/src/lib/CMakeFiles/git_matrix.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/lib/CMakeFiles/git_matrix.dir/depend

