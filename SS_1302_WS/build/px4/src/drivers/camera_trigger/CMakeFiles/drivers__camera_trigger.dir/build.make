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
include src/drivers/camera_trigger/CMakeFiles/drivers__camera_trigger.dir/depend.make

# Include the progress variables for this target.
include src/drivers/camera_trigger/CMakeFiles/drivers__camera_trigger.dir/progress.make

# Include the compile flags for this target's objects.
include src/drivers/camera_trigger/CMakeFiles/drivers__camera_trigger.dir/flags.make

src/drivers/camera_trigger/CMakeFiles/drivers__camera_trigger.dir/camera_trigger.cpp.o: src/drivers/camera_trigger/CMakeFiles/drivers__camera_trigger.dir/flags.make
src/drivers/camera_trigger/CMakeFiles/drivers__camera_trigger.dir/camera_trigger.cpp.o: /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot/src/drivers/camera_trigger/camera_trigger.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/drivers/camera_trigger/CMakeFiles/drivers__camera_trigger.dir/camera_trigger.cpp.o"
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/src/drivers/camera_trigger && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drivers__camera_trigger.dir/camera_trigger.cpp.o -c /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot/src/drivers/camera_trigger/camera_trigger.cpp

src/drivers/camera_trigger/CMakeFiles/drivers__camera_trigger.dir/camera_trigger.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drivers__camera_trigger.dir/camera_trigger.cpp.i"
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/src/drivers/camera_trigger && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot/src/drivers/camera_trigger/camera_trigger.cpp > CMakeFiles/drivers__camera_trigger.dir/camera_trigger.cpp.i

src/drivers/camera_trigger/CMakeFiles/drivers__camera_trigger.dir/camera_trigger.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drivers__camera_trigger.dir/camera_trigger.cpp.s"
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/src/drivers/camera_trigger && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot/src/drivers/camera_trigger/camera_trigger.cpp -o CMakeFiles/drivers__camera_trigger.dir/camera_trigger.cpp.s

src/drivers/camera_trigger/CMakeFiles/drivers__camera_trigger.dir/interfaces/src/camera_interface.cpp.o: src/drivers/camera_trigger/CMakeFiles/drivers__camera_trigger.dir/flags.make
src/drivers/camera_trigger/CMakeFiles/drivers__camera_trigger.dir/interfaces/src/camera_interface.cpp.o: /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot/src/drivers/camera_trigger/interfaces/src/camera_interface.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/drivers/camera_trigger/CMakeFiles/drivers__camera_trigger.dir/interfaces/src/camera_interface.cpp.o"
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/src/drivers/camera_trigger && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drivers__camera_trigger.dir/interfaces/src/camera_interface.cpp.o -c /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot/src/drivers/camera_trigger/interfaces/src/camera_interface.cpp

src/drivers/camera_trigger/CMakeFiles/drivers__camera_trigger.dir/interfaces/src/camera_interface.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drivers__camera_trigger.dir/interfaces/src/camera_interface.cpp.i"
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/src/drivers/camera_trigger && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot/src/drivers/camera_trigger/interfaces/src/camera_interface.cpp > CMakeFiles/drivers__camera_trigger.dir/interfaces/src/camera_interface.cpp.i

src/drivers/camera_trigger/CMakeFiles/drivers__camera_trigger.dir/interfaces/src/camera_interface.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drivers__camera_trigger.dir/interfaces/src/camera_interface.cpp.s"
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/src/drivers/camera_trigger && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot/src/drivers/camera_trigger/interfaces/src/camera_interface.cpp -o CMakeFiles/drivers__camera_trigger.dir/interfaces/src/camera_interface.cpp.s

src/drivers/camera_trigger/CMakeFiles/drivers__camera_trigger.dir/interfaces/src/pwm.cpp.o: src/drivers/camera_trigger/CMakeFiles/drivers__camera_trigger.dir/flags.make
src/drivers/camera_trigger/CMakeFiles/drivers__camera_trigger.dir/interfaces/src/pwm.cpp.o: /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot/src/drivers/camera_trigger/interfaces/src/pwm.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/drivers/camera_trigger/CMakeFiles/drivers__camera_trigger.dir/interfaces/src/pwm.cpp.o"
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/src/drivers/camera_trigger && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drivers__camera_trigger.dir/interfaces/src/pwm.cpp.o -c /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot/src/drivers/camera_trigger/interfaces/src/pwm.cpp

src/drivers/camera_trigger/CMakeFiles/drivers__camera_trigger.dir/interfaces/src/pwm.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drivers__camera_trigger.dir/interfaces/src/pwm.cpp.i"
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/src/drivers/camera_trigger && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot/src/drivers/camera_trigger/interfaces/src/pwm.cpp > CMakeFiles/drivers__camera_trigger.dir/interfaces/src/pwm.cpp.i

src/drivers/camera_trigger/CMakeFiles/drivers__camera_trigger.dir/interfaces/src/pwm.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drivers__camera_trigger.dir/interfaces/src/pwm.cpp.s"
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/src/drivers/camera_trigger && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot/src/drivers/camera_trigger/interfaces/src/pwm.cpp -o CMakeFiles/drivers__camera_trigger.dir/interfaces/src/pwm.cpp.s

src/drivers/camera_trigger/CMakeFiles/drivers__camera_trigger.dir/interfaces/src/seagull_map2.cpp.o: src/drivers/camera_trigger/CMakeFiles/drivers__camera_trigger.dir/flags.make
src/drivers/camera_trigger/CMakeFiles/drivers__camera_trigger.dir/interfaces/src/seagull_map2.cpp.o: /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot/src/drivers/camera_trigger/interfaces/src/seagull_map2.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object src/drivers/camera_trigger/CMakeFiles/drivers__camera_trigger.dir/interfaces/src/seagull_map2.cpp.o"
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/src/drivers/camera_trigger && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drivers__camera_trigger.dir/interfaces/src/seagull_map2.cpp.o -c /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot/src/drivers/camera_trigger/interfaces/src/seagull_map2.cpp

src/drivers/camera_trigger/CMakeFiles/drivers__camera_trigger.dir/interfaces/src/seagull_map2.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drivers__camera_trigger.dir/interfaces/src/seagull_map2.cpp.i"
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/src/drivers/camera_trigger && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot/src/drivers/camera_trigger/interfaces/src/seagull_map2.cpp > CMakeFiles/drivers__camera_trigger.dir/interfaces/src/seagull_map2.cpp.i

src/drivers/camera_trigger/CMakeFiles/drivers__camera_trigger.dir/interfaces/src/seagull_map2.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drivers__camera_trigger.dir/interfaces/src/seagull_map2.cpp.s"
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/src/drivers/camera_trigger && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot/src/drivers/camera_trigger/interfaces/src/seagull_map2.cpp -o CMakeFiles/drivers__camera_trigger.dir/interfaces/src/seagull_map2.cpp.s

src/drivers/camera_trigger/CMakeFiles/drivers__camera_trigger.dir/interfaces/src/gpio.cpp.o: src/drivers/camera_trigger/CMakeFiles/drivers__camera_trigger.dir/flags.make
src/drivers/camera_trigger/CMakeFiles/drivers__camera_trigger.dir/interfaces/src/gpio.cpp.o: /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot/src/drivers/camera_trigger/interfaces/src/gpio.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object src/drivers/camera_trigger/CMakeFiles/drivers__camera_trigger.dir/interfaces/src/gpio.cpp.o"
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/src/drivers/camera_trigger && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drivers__camera_trigger.dir/interfaces/src/gpio.cpp.o -c /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot/src/drivers/camera_trigger/interfaces/src/gpio.cpp

src/drivers/camera_trigger/CMakeFiles/drivers__camera_trigger.dir/interfaces/src/gpio.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drivers__camera_trigger.dir/interfaces/src/gpio.cpp.i"
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/src/drivers/camera_trigger && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot/src/drivers/camera_trigger/interfaces/src/gpio.cpp > CMakeFiles/drivers__camera_trigger.dir/interfaces/src/gpio.cpp.i

src/drivers/camera_trigger/CMakeFiles/drivers__camera_trigger.dir/interfaces/src/gpio.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drivers__camera_trigger.dir/interfaces/src/gpio.cpp.s"
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/src/drivers/camera_trigger && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot/src/drivers/camera_trigger/interfaces/src/gpio.cpp -o CMakeFiles/drivers__camera_trigger.dir/interfaces/src/gpio.cpp.s

# Object files for target drivers__camera_trigger
drivers__camera_trigger_OBJECTS = \
"CMakeFiles/drivers__camera_trigger.dir/camera_trigger.cpp.o" \
"CMakeFiles/drivers__camera_trigger.dir/interfaces/src/camera_interface.cpp.o" \
"CMakeFiles/drivers__camera_trigger.dir/interfaces/src/pwm.cpp.o" \
"CMakeFiles/drivers__camera_trigger.dir/interfaces/src/seagull_map2.cpp.o" \
"CMakeFiles/drivers__camera_trigger.dir/interfaces/src/gpio.cpp.o"

# External object files for target drivers__camera_trigger
drivers__camera_trigger_EXTERNAL_OBJECTS =

/home/ch13f_1419/E-Yantra/SS_1302_WS/devel/.private/px4/lib/libdrivers__camera_trigger.a: src/drivers/camera_trigger/CMakeFiles/drivers__camera_trigger.dir/camera_trigger.cpp.o
/home/ch13f_1419/E-Yantra/SS_1302_WS/devel/.private/px4/lib/libdrivers__camera_trigger.a: src/drivers/camera_trigger/CMakeFiles/drivers__camera_trigger.dir/interfaces/src/camera_interface.cpp.o
/home/ch13f_1419/E-Yantra/SS_1302_WS/devel/.private/px4/lib/libdrivers__camera_trigger.a: src/drivers/camera_trigger/CMakeFiles/drivers__camera_trigger.dir/interfaces/src/pwm.cpp.o
/home/ch13f_1419/E-Yantra/SS_1302_WS/devel/.private/px4/lib/libdrivers__camera_trigger.a: src/drivers/camera_trigger/CMakeFiles/drivers__camera_trigger.dir/interfaces/src/seagull_map2.cpp.o
/home/ch13f_1419/E-Yantra/SS_1302_WS/devel/.private/px4/lib/libdrivers__camera_trigger.a: src/drivers/camera_trigger/CMakeFiles/drivers__camera_trigger.dir/interfaces/src/gpio.cpp.o
/home/ch13f_1419/E-Yantra/SS_1302_WS/devel/.private/px4/lib/libdrivers__camera_trigger.a: src/drivers/camera_trigger/CMakeFiles/drivers__camera_trigger.dir/build.make
/home/ch13f_1419/E-Yantra/SS_1302_WS/devel/.private/px4/lib/libdrivers__camera_trigger.a: src/drivers/camera_trigger/CMakeFiles/drivers__camera_trigger.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX static library /home/ch13f_1419/E-Yantra/SS_1302_WS/devel/.private/px4/lib/libdrivers__camera_trigger.a"
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/src/drivers/camera_trigger && $(CMAKE_COMMAND) -P CMakeFiles/drivers__camera_trigger.dir/cmake_clean_target.cmake
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/src/drivers/camera_trigger && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/drivers__camera_trigger.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/drivers/camera_trigger/CMakeFiles/drivers__camera_trigger.dir/build: /home/ch13f_1419/E-Yantra/SS_1302_WS/devel/.private/px4/lib/libdrivers__camera_trigger.a

.PHONY : src/drivers/camera_trigger/CMakeFiles/drivers__camera_trigger.dir/build

src/drivers/camera_trigger/CMakeFiles/drivers__camera_trigger.dir/clean:
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/src/drivers/camera_trigger && $(CMAKE_COMMAND) -P CMakeFiles/drivers__camera_trigger.dir/cmake_clean.cmake
.PHONY : src/drivers/camera_trigger/CMakeFiles/drivers__camera_trigger.dir/clean

src/drivers/camera_trigger/CMakeFiles/drivers__camera_trigger.dir/depend:
	cd /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot/src/drivers/camera_trigger /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4 /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/src/drivers/camera_trigger /home/ch13f_1419/E-Yantra/SS_1302_WS/build/px4/src/drivers/camera_trigger/CMakeFiles/drivers__camera_trigger.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/drivers/camera_trigger/CMakeFiles/drivers__camera_trigger.dir/depend

