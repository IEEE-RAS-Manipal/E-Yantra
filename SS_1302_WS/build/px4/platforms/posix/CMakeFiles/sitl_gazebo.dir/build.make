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

# Utility rule file for sitl_gazebo.

# Include the progress variables for this target.
include platforms/posix/CMakeFiles/sitl_gazebo.dir/progress.make

platforms/posix/CMakeFiles/sitl_gazebo: platforms/posix/CMakeFiles/sitl_gazebo-complete


platforms/posix/CMakeFiles/sitl_gazebo-complete: external/Stamp/sitl_gazebo/sitl_gazebo-install
platforms/posix/CMakeFiles/sitl_gazebo-complete: external/Stamp/sitl_gazebo/sitl_gazebo-mkdir
platforms/posix/CMakeFiles/sitl_gazebo-complete: external/Stamp/sitl_gazebo/sitl_gazebo-download
platforms/posix/CMakeFiles/sitl_gazebo-complete: external/Stamp/sitl_gazebo/sitl_gazebo-update
platforms/posix/CMakeFiles/sitl_gazebo-complete: external/Stamp/sitl_gazebo/sitl_gazebo-patch
platforms/posix/CMakeFiles/sitl_gazebo-complete: external/Stamp/sitl_gazebo/sitl_gazebo-configure
platforms/posix/CMakeFiles/sitl_gazebo-complete: external/Stamp/sitl_gazebo/sitl_gazebo-build
platforms/posix/CMakeFiles/sitl_gazebo-complete: external/Stamp/sitl_gazebo/sitl_gazebo-install
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Completed 'sitl_gazebo'"
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/platforms/posix && /usr/bin/cmake -E make_directory /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/platforms/posix/CMakeFiles
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/platforms/posix && /usr/bin/cmake -E touch /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/platforms/posix/CMakeFiles/sitl_gazebo-complete
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/platforms/posix && /usr/bin/cmake -E touch /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/external/Stamp/sitl_gazebo/sitl_gazebo-done

external/Stamp/sitl_gazebo/sitl_gazebo-install: external/Stamp/sitl_gazebo/sitl_gazebo-build
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "No install step for 'sitl_gazebo'"
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/build_gazebo && /usr/bin/cmake -E echo_append
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/build_gazebo && /usr/bin/cmake -E touch /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/external/Stamp/sitl_gazebo/sitl_gazebo-install

external/Stamp/sitl_gazebo/sitl_gazebo-mkdir:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Creating directories for 'sitl_gazebo'"
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/platforms/posix && /usr/bin/cmake -E make_directory /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot/Tools/sitl_gazebo
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/platforms/posix && /usr/bin/cmake -E make_directory /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/build_gazebo
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/platforms/posix && /usr/bin/cmake -E make_directory /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/external/Install/sitl_gazebo
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/platforms/posix && /usr/bin/cmake -E make_directory /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/external/tmp/sitl_gazebo
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/platforms/posix && /usr/bin/cmake -E make_directory /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/external/Stamp/sitl_gazebo
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/platforms/posix && /usr/bin/cmake -E make_directory /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/external/Download/sitl_gazebo
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/platforms/posix && /usr/bin/cmake -E make_directory /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/external/Stamp/sitl_gazebo
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/platforms/posix && /usr/bin/cmake -E touch /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/external/Stamp/sitl_gazebo/sitl_gazebo-mkdir

external/Stamp/sitl_gazebo/sitl_gazebo-download: external/Stamp/sitl_gazebo/sitl_gazebo-mkdir
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "No download step for 'sitl_gazebo'"
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/platforms/posix && /usr/bin/cmake -E echo_append
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/platforms/posix && /usr/bin/cmake -E touch /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/external/Stamp/sitl_gazebo/sitl_gazebo-download

external/Stamp/sitl_gazebo/sitl_gazebo-update: external/Stamp/sitl_gazebo/sitl_gazebo-download
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "No update step for 'sitl_gazebo'"
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/platforms/posix && /usr/bin/cmake -E echo_append
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/platforms/posix && /usr/bin/cmake -E touch /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/external/Stamp/sitl_gazebo/sitl_gazebo-update

external/Stamp/sitl_gazebo/sitl_gazebo-patch: external/Stamp/sitl_gazebo/sitl_gazebo-download
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "No patch step for 'sitl_gazebo'"
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/platforms/posix && /usr/bin/cmake -E echo_append
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/platforms/posix && /usr/bin/cmake -E touch /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/external/Stamp/sitl_gazebo/sitl_gazebo-patch

external/Stamp/sitl_gazebo/sitl_gazebo-configure: external/tmp/sitl_gazebo/sitl_gazebo-cfgcmd.txt
external/Stamp/sitl_gazebo/sitl_gazebo-configure: external/Stamp/sitl_gazebo/sitl_gazebo-update
external/Stamp/sitl_gazebo/sitl_gazebo-configure: external/Stamp/sitl_gazebo/sitl_gazebo-patch
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Performing configure step for 'sitl_gazebo'"
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/build_gazebo && /usr/bin/cmake -DCMAKE_INSTALL_PREFIX=/home/ch13f_1419/E-Yantra/SS_1302_WS/install -DSEND_ODOMETRY_DATA=ON -DGENERATE_ROS_MODELS=ON "-GUnix Makefiles" /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot/Tools/sitl_gazebo
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/build_gazebo && /usr/bin/cmake -E touch /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/external/Stamp/sitl_gazebo/sitl_gazebo-configure

external/Stamp/sitl_gazebo/sitl_gazebo-build: external/Stamp/sitl_gazebo/sitl_gazebo-configure
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Performing build step for 'sitl_gazebo'"
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/build_gazebo && /usr/bin/cmake --build /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/build_gazebo -- -j 7

sitl_gazebo: platforms/posix/CMakeFiles/sitl_gazebo
sitl_gazebo: platforms/posix/CMakeFiles/sitl_gazebo-complete
sitl_gazebo: external/Stamp/sitl_gazebo/sitl_gazebo-install
sitl_gazebo: external/Stamp/sitl_gazebo/sitl_gazebo-mkdir
sitl_gazebo: external/Stamp/sitl_gazebo/sitl_gazebo-download
sitl_gazebo: external/Stamp/sitl_gazebo/sitl_gazebo-update
sitl_gazebo: external/Stamp/sitl_gazebo/sitl_gazebo-patch
sitl_gazebo: external/Stamp/sitl_gazebo/sitl_gazebo-configure
sitl_gazebo: external/Stamp/sitl_gazebo/sitl_gazebo-build
sitl_gazebo: platforms/posix/CMakeFiles/sitl_gazebo.dir/build.make

.PHONY : sitl_gazebo

# Rule to build all files generated by this target.
platforms/posix/CMakeFiles/sitl_gazebo.dir/build: sitl_gazebo

.PHONY : platforms/posix/CMakeFiles/sitl_gazebo.dir/build

platforms/posix/CMakeFiles/sitl_gazebo.dir/clean:
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/platforms/posix && $(CMAKE_COMMAND) -P CMakeFiles/sitl_gazebo.dir/cmake_clean.cmake
.PHONY : platforms/posix/CMakeFiles/sitl_gazebo.dir/clean

platforms/posix/CMakeFiles/sitl_gazebo.dir/depend:
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot/platforms/posix /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4 /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/platforms/posix /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/platforms/posix/CMakeFiles/sitl_gazebo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : platforms/posix/CMakeFiles/sitl_gazebo.dir/depend

