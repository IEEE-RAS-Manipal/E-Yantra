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

# Utility rule file for git_jmavsim.

# Include the progress variables for this target.
include platforms/posix/CMakeFiles/git_jmavsim.dir/progress.make

platforms/posix/CMakeFiles/git_jmavsim: platforms/posix/git_init__home_ch13f_1419_E-Yantra_SS_1302_WS_src_PX4-Autopilot_Tools_jMAVSim.stamp


platforms/posix/git_init__home_ch13f_1419_E-Yantra_SS_1302_WS_src_PX4-Autopilot_Tools_jMAVSim.stamp: /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot/.gitmodules
platforms/posix/git_init__home_ch13f_1419_E-Yantra_SS_1302_WS_src_PX4-Autopilot_Tools_jMAVSim.stamp: /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot/Tools/jMAVSim/.git
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "git submodule Tools/jMAVSim"
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot && Tools/check_submodules.sh Tools/jMAVSim
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot && /usr/bin/cmake -E touch /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/platforms/posix/git_init__home_ch13f_1419_E-Yantra_SS_1302_WS_src_PX4-Autopilot_Tools_jMAVSim.stamp

git_jmavsim: platforms/posix/CMakeFiles/git_jmavsim
git_jmavsim: platforms/posix/git_init__home_ch13f_1419_E-Yantra_SS_1302_WS_src_PX4-Autopilot_Tools_jMAVSim.stamp
git_jmavsim: platforms/posix/CMakeFiles/git_jmavsim.dir/build.make

.PHONY : git_jmavsim

# Rule to build all files generated by this target.
platforms/posix/CMakeFiles/git_jmavsim.dir/build: git_jmavsim

.PHONY : platforms/posix/CMakeFiles/git_jmavsim.dir/build

platforms/posix/CMakeFiles/git_jmavsim.dir/clean:
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/platforms/posix && $(CMAKE_COMMAND) -P CMakeFiles/git_jmavsim.dir/cmake_clean.cmake
.PHONY : platforms/posix/CMakeFiles/git_jmavsim.dir/clean

platforms/posix/CMakeFiles/git_jmavsim.dir/depend:
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot/platforms/posix /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4 /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/platforms/posix /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/platforms/posix/CMakeFiles/git_jmavsim.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : platforms/posix/CMakeFiles/git_jmavsim.dir/depend

