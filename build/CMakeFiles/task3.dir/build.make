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
CMAKE_SOURCE_DIR = /home/leo/UR5-project/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/leo/UR5-project/build

# Include any dependencies generated for this target.
include CMakeFiles/task3.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/task3.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/task3.dir/flags.make

CMakeFiles/task3.dir/src/tasks/task3.cpp.o: CMakeFiles/task3.dir/flags.make
CMakeFiles/task3.dir/src/tasks/task3.cpp.o: /home/leo/UR5-project/src/src/tasks/task3.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leo/UR5-project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/task3.dir/src/tasks/task3.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/task3.dir/src/tasks/task3.cpp.o -c /home/leo/UR5-project/src/src/tasks/task3.cpp

CMakeFiles/task3.dir/src/tasks/task3.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/task3.dir/src/tasks/task3.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leo/UR5-project/src/src/tasks/task3.cpp > CMakeFiles/task3.dir/src/tasks/task3.cpp.i

CMakeFiles/task3.dir/src/tasks/task3.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/task3.dir/src/tasks/task3.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leo/UR5-project/src/src/tasks/task3.cpp -o CMakeFiles/task3.dir/src/tasks/task3.cpp.s

CMakeFiles/task3.dir/src/Bezier/Bezier.cpp.o: CMakeFiles/task3.dir/flags.make
CMakeFiles/task3.dir/src/Bezier/Bezier.cpp.o: /home/leo/UR5-project/src/src/Bezier/Bezier.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leo/UR5-project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/task3.dir/src/Bezier/Bezier.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/task3.dir/src/Bezier/Bezier.cpp.o -c /home/leo/UR5-project/src/src/Bezier/Bezier.cpp

CMakeFiles/task3.dir/src/Bezier/Bezier.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/task3.dir/src/Bezier/Bezier.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leo/UR5-project/src/src/Bezier/Bezier.cpp > CMakeFiles/task3.dir/src/Bezier/Bezier.cpp.i

CMakeFiles/task3.dir/src/Bezier/Bezier.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/task3.dir/src/Bezier/Bezier.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leo/UR5-project/src/src/Bezier/Bezier.cpp -o CMakeFiles/task3.dir/src/Bezier/Bezier.cpp.s

CMakeFiles/task3.dir/src/Bezier/QuinticBezier.cpp.o: CMakeFiles/task3.dir/flags.make
CMakeFiles/task3.dir/src/Bezier/QuinticBezier.cpp.o: /home/leo/UR5-project/src/src/Bezier/QuinticBezier.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leo/UR5-project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/task3.dir/src/Bezier/QuinticBezier.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/task3.dir/src/Bezier/QuinticBezier.cpp.o -c /home/leo/UR5-project/src/src/Bezier/QuinticBezier.cpp

CMakeFiles/task3.dir/src/Bezier/QuinticBezier.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/task3.dir/src/Bezier/QuinticBezier.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leo/UR5-project/src/src/Bezier/QuinticBezier.cpp > CMakeFiles/task3.dir/src/Bezier/QuinticBezier.cpp.i

CMakeFiles/task3.dir/src/Bezier/QuinticBezier.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/task3.dir/src/Bezier/QuinticBezier.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leo/UR5-project/src/src/Bezier/QuinticBezier.cpp -o CMakeFiles/task3.dir/src/Bezier/QuinticBezier.cpp.s

CMakeFiles/task3.dir/src/Checkpoint/Checkpoint.cpp.o: CMakeFiles/task3.dir/flags.make
CMakeFiles/task3.dir/src/Checkpoint/Checkpoint.cpp.o: /home/leo/UR5-project/src/src/Checkpoint/Checkpoint.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leo/UR5-project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/task3.dir/src/Checkpoint/Checkpoint.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/task3.dir/src/Checkpoint/Checkpoint.cpp.o -c /home/leo/UR5-project/src/src/Checkpoint/Checkpoint.cpp

CMakeFiles/task3.dir/src/Checkpoint/Checkpoint.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/task3.dir/src/Checkpoint/Checkpoint.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leo/UR5-project/src/src/Checkpoint/Checkpoint.cpp > CMakeFiles/task3.dir/src/Checkpoint/Checkpoint.cpp.i

CMakeFiles/task3.dir/src/Checkpoint/Checkpoint.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/task3.dir/src/Checkpoint/Checkpoint.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leo/UR5-project/src/src/Checkpoint/Checkpoint.cpp -o CMakeFiles/task3.dir/src/Checkpoint/Checkpoint.cpp.s

CMakeFiles/task3.dir/src/ObstacleAvoidance/Funnel.cpp.o: CMakeFiles/task3.dir/flags.make
CMakeFiles/task3.dir/src/ObstacleAvoidance/Funnel.cpp.o: /home/leo/UR5-project/src/src/ObstacleAvoidance/Funnel.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leo/UR5-project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/task3.dir/src/ObstacleAvoidance/Funnel.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/task3.dir/src/ObstacleAvoidance/Funnel.cpp.o -c /home/leo/UR5-project/src/src/ObstacleAvoidance/Funnel.cpp

CMakeFiles/task3.dir/src/ObstacleAvoidance/Funnel.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/task3.dir/src/ObstacleAvoidance/Funnel.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leo/UR5-project/src/src/ObstacleAvoidance/Funnel.cpp > CMakeFiles/task3.dir/src/ObstacleAvoidance/Funnel.cpp.i

CMakeFiles/task3.dir/src/ObstacleAvoidance/Funnel.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/task3.dir/src/ObstacleAvoidance/Funnel.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leo/UR5-project/src/src/ObstacleAvoidance/Funnel.cpp -o CMakeFiles/task3.dir/src/ObstacleAvoidance/Funnel.cpp.s

CMakeFiles/task3.dir/src/ObstacleAvoidance/Hill.cpp.o: CMakeFiles/task3.dir/flags.make
CMakeFiles/task3.dir/src/ObstacleAvoidance/Hill.cpp.o: /home/leo/UR5-project/src/src/ObstacleAvoidance/Hill.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leo/UR5-project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/task3.dir/src/ObstacleAvoidance/Hill.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/task3.dir/src/ObstacleAvoidance/Hill.cpp.o -c /home/leo/UR5-project/src/src/ObstacleAvoidance/Hill.cpp

CMakeFiles/task3.dir/src/ObstacleAvoidance/Hill.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/task3.dir/src/ObstacleAvoidance/Hill.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leo/UR5-project/src/src/ObstacleAvoidance/Hill.cpp > CMakeFiles/task3.dir/src/ObstacleAvoidance/Hill.cpp.i

CMakeFiles/task3.dir/src/ObstacleAvoidance/Hill.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/task3.dir/src/ObstacleAvoidance/Hill.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leo/UR5-project/src/src/ObstacleAvoidance/Hill.cpp -o CMakeFiles/task3.dir/src/ObstacleAvoidance/Hill.cpp.s

CMakeFiles/task3.dir/src/ObstacleAvoidance/Obstacle.cpp.o: CMakeFiles/task3.dir/flags.make
CMakeFiles/task3.dir/src/ObstacleAvoidance/Obstacle.cpp.o: /home/leo/UR5-project/src/src/ObstacleAvoidance/Obstacle.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leo/UR5-project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/task3.dir/src/ObstacleAvoidance/Obstacle.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/task3.dir/src/ObstacleAvoidance/Obstacle.cpp.o -c /home/leo/UR5-project/src/src/ObstacleAvoidance/Obstacle.cpp

CMakeFiles/task3.dir/src/ObstacleAvoidance/Obstacle.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/task3.dir/src/ObstacleAvoidance/Obstacle.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leo/UR5-project/src/src/ObstacleAvoidance/Obstacle.cpp > CMakeFiles/task3.dir/src/ObstacleAvoidance/Obstacle.cpp.i

CMakeFiles/task3.dir/src/ObstacleAvoidance/Obstacle.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/task3.dir/src/ObstacleAvoidance/Obstacle.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leo/UR5-project/src/src/ObstacleAvoidance/Obstacle.cpp -o CMakeFiles/task3.dir/src/ObstacleAvoidance/Obstacle.cpp.s

CMakeFiles/task3.dir/src/ObstacleAvoidance/ObstacleAvoidance.cpp.o: CMakeFiles/task3.dir/flags.make
CMakeFiles/task3.dir/src/ObstacleAvoidance/ObstacleAvoidance.cpp.o: /home/leo/UR5-project/src/src/ObstacleAvoidance/ObstacleAvoidance.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leo/UR5-project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/task3.dir/src/ObstacleAvoidance/ObstacleAvoidance.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/task3.dir/src/ObstacleAvoidance/ObstacleAvoidance.cpp.o -c /home/leo/UR5-project/src/src/ObstacleAvoidance/ObstacleAvoidance.cpp

CMakeFiles/task3.dir/src/ObstacleAvoidance/ObstacleAvoidance.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/task3.dir/src/ObstacleAvoidance/ObstacleAvoidance.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leo/UR5-project/src/src/ObstacleAvoidance/ObstacleAvoidance.cpp > CMakeFiles/task3.dir/src/ObstacleAvoidance/ObstacleAvoidance.cpp.i

CMakeFiles/task3.dir/src/ObstacleAvoidance/ObstacleAvoidance.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/task3.dir/src/ObstacleAvoidance/ObstacleAvoidance.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leo/UR5-project/src/src/ObstacleAvoidance/ObstacleAvoidance.cpp -o CMakeFiles/task3.dir/src/ObstacleAvoidance/ObstacleAvoidance.cpp.s

CMakeFiles/task3.dir/src/Spline/Spline.cpp.o: CMakeFiles/task3.dir/flags.make
CMakeFiles/task3.dir/src/Spline/Spline.cpp.o: /home/leo/UR5-project/src/src/Spline/Spline.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leo/UR5-project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/task3.dir/src/Spline/Spline.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/task3.dir/src/Spline/Spline.cpp.o -c /home/leo/UR5-project/src/src/Spline/Spline.cpp

CMakeFiles/task3.dir/src/Spline/Spline.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/task3.dir/src/Spline/Spline.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leo/UR5-project/src/src/Spline/Spline.cpp > CMakeFiles/task3.dir/src/Spline/Spline.cpp.i

CMakeFiles/task3.dir/src/Spline/Spline.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/task3.dir/src/Spline/Spline.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leo/UR5-project/src/src/Spline/Spline.cpp -o CMakeFiles/task3.dir/src/Spline/Spline.cpp.s

CMakeFiles/task3.dir/src/Trajectory/LinearTrajectory.cpp.o: CMakeFiles/task3.dir/flags.make
CMakeFiles/task3.dir/src/Trajectory/LinearTrajectory.cpp.o: /home/leo/UR5-project/src/src/Trajectory/LinearTrajectory.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leo/UR5-project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object CMakeFiles/task3.dir/src/Trajectory/LinearTrajectory.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/task3.dir/src/Trajectory/LinearTrajectory.cpp.o -c /home/leo/UR5-project/src/src/Trajectory/LinearTrajectory.cpp

CMakeFiles/task3.dir/src/Trajectory/LinearTrajectory.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/task3.dir/src/Trajectory/LinearTrajectory.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leo/UR5-project/src/src/Trajectory/LinearTrajectory.cpp > CMakeFiles/task3.dir/src/Trajectory/LinearTrajectory.cpp.i

CMakeFiles/task3.dir/src/Trajectory/LinearTrajectory.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/task3.dir/src/Trajectory/LinearTrajectory.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leo/UR5-project/src/src/Trajectory/LinearTrajectory.cpp -o CMakeFiles/task3.dir/src/Trajectory/LinearTrajectory.cpp.s

CMakeFiles/task3.dir/src/Trajectory/MoveObjectTrajectory.cpp.o: CMakeFiles/task3.dir/flags.make
CMakeFiles/task3.dir/src/Trajectory/MoveObjectTrajectory.cpp.o: /home/leo/UR5-project/src/src/Trajectory/MoveObjectTrajectory.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leo/UR5-project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object CMakeFiles/task3.dir/src/Trajectory/MoveObjectTrajectory.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/task3.dir/src/Trajectory/MoveObjectTrajectory.cpp.o -c /home/leo/UR5-project/src/src/Trajectory/MoveObjectTrajectory.cpp

CMakeFiles/task3.dir/src/Trajectory/MoveObjectTrajectory.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/task3.dir/src/Trajectory/MoveObjectTrajectory.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leo/UR5-project/src/src/Trajectory/MoveObjectTrajectory.cpp > CMakeFiles/task3.dir/src/Trajectory/MoveObjectTrajectory.cpp.i

CMakeFiles/task3.dir/src/Trajectory/MoveObjectTrajectory.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/task3.dir/src/Trajectory/MoveObjectTrajectory.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leo/UR5-project/src/src/Trajectory/MoveObjectTrajectory.cpp -o CMakeFiles/task3.dir/src/Trajectory/MoveObjectTrajectory.cpp.s

CMakeFiles/task3.dir/src/Trajectory/Trajectory.cpp.o: CMakeFiles/task3.dir/flags.make
CMakeFiles/task3.dir/src/Trajectory/Trajectory.cpp.o: /home/leo/UR5-project/src/src/Trajectory/Trajectory.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leo/UR5-project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building CXX object CMakeFiles/task3.dir/src/Trajectory/Trajectory.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/task3.dir/src/Trajectory/Trajectory.cpp.o -c /home/leo/UR5-project/src/src/Trajectory/Trajectory.cpp

CMakeFiles/task3.dir/src/Trajectory/Trajectory.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/task3.dir/src/Trajectory/Trajectory.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leo/UR5-project/src/src/Trajectory/Trajectory.cpp > CMakeFiles/task3.dir/src/Trajectory/Trajectory.cpp.i

CMakeFiles/task3.dir/src/Trajectory/Trajectory.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/task3.dir/src/Trajectory/Trajectory.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leo/UR5-project/src/src/Trajectory/Trajectory.cpp -o CMakeFiles/task3.dir/src/Trajectory/Trajectory.cpp.s

CMakeFiles/task3.dir/src/UR5/UR5.cpp.o: CMakeFiles/task3.dir/flags.make
CMakeFiles/task3.dir/src/UR5/UR5.cpp.o: /home/leo/UR5-project/src/src/UR5/UR5.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leo/UR5-project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Building CXX object CMakeFiles/task3.dir/src/UR5/UR5.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/task3.dir/src/UR5/UR5.cpp.o -c /home/leo/UR5-project/src/src/UR5/UR5.cpp

CMakeFiles/task3.dir/src/UR5/UR5.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/task3.dir/src/UR5/UR5.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leo/UR5-project/src/src/UR5/UR5.cpp > CMakeFiles/task3.dir/src/UR5/UR5.cpp.i

CMakeFiles/task3.dir/src/UR5/UR5.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/task3.dir/src/UR5/UR5.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leo/UR5-project/src/src/UR5/UR5.cpp -o CMakeFiles/task3.dir/src/UR5/UR5.cpp.s

CMakeFiles/task3.dir/src/tasks/rotations.cpp.o: CMakeFiles/task3.dir/flags.make
CMakeFiles/task3.dir/src/tasks/rotations.cpp.o: /home/leo/UR5-project/src/src/tasks/rotations.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leo/UR5-project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Building CXX object CMakeFiles/task3.dir/src/tasks/rotations.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/task3.dir/src/tasks/rotations.cpp.o -c /home/leo/UR5-project/src/src/tasks/rotations.cpp

CMakeFiles/task3.dir/src/tasks/rotations.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/task3.dir/src/tasks/rotations.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leo/UR5-project/src/src/tasks/rotations.cpp > CMakeFiles/task3.dir/src/tasks/rotations.cpp.i

CMakeFiles/task3.dir/src/tasks/rotations.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/task3.dir/src/tasks/rotations.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leo/UR5-project/src/src/tasks/rotations.cpp -o CMakeFiles/task3.dir/src/tasks/rotations.cpp.s

CMakeFiles/task3.dir/src/utilities/utilities.cpp.o: CMakeFiles/task3.dir/flags.make
CMakeFiles/task3.dir/src/utilities/utilities.cpp.o: /home/leo/UR5-project/src/src/utilities/utilities.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leo/UR5-project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_15) "Building CXX object CMakeFiles/task3.dir/src/utilities/utilities.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/task3.dir/src/utilities/utilities.cpp.o -c /home/leo/UR5-project/src/src/utilities/utilities.cpp

CMakeFiles/task3.dir/src/utilities/utilities.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/task3.dir/src/utilities/utilities.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leo/UR5-project/src/src/utilities/utilities.cpp > CMakeFiles/task3.dir/src/utilities/utilities.cpp.i

CMakeFiles/task3.dir/src/utilities/utilities.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/task3.dir/src/utilities/utilities.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leo/UR5-project/src/src/utilities/utilities.cpp -o CMakeFiles/task3.dir/src/utilities/utilities.cpp.s

CMakeFiles/task3.dir/src/visionClient/visionClient.cpp.o: CMakeFiles/task3.dir/flags.make
CMakeFiles/task3.dir/src/visionClient/visionClient.cpp.o: /home/leo/UR5-project/src/src/visionClient/visionClient.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leo/UR5-project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_16) "Building CXX object CMakeFiles/task3.dir/src/visionClient/visionClient.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/task3.dir/src/visionClient/visionClient.cpp.o -c /home/leo/UR5-project/src/src/visionClient/visionClient.cpp

CMakeFiles/task3.dir/src/visionClient/visionClient.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/task3.dir/src/visionClient/visionClient.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leo/UR5-project/src/src/visionClient/visionClient.cpp > CMakeFiles/task3.dir/src/visionClient/visionClient.cpp.i

CMakeFiles/task3.dir/src/visionClient/visionClient.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/task3.dir/src/visionClient/visionClient.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leo/UR5-project/src/src/visionClient/visionClient.cpp -o CMakeFiles/task3.dir/src/visionClient/visionClient.cpp.s

# Object files for target task3
task3_OBJECTS = \
"CMakeFiles/task3.dir/src/tasks/task3.cpp.o" \
"CMakeFiles/task3.dir/src/Bezier/Bezier.cpp.o" \
"CMakeFiles/task3.dir/src/Bezier/QuinticBezier.cpp.o" \
"CMakeFiles/task3.dir/src/Checkpoint/Checkpoint.cpp.o" \
"CMakeFiles/task3.dir/src/ObstacleAvoidance/Funnel.cpp.o" \
"CMakeFiles/task3.dir/src/ObstacleAvoidance/Hill.cpp.o" \
"CMakeFiles/task3.dir/src/ObstacleAvoidance/Obstacle.cpp.o" \
"CMakeFiles/task3.dir/src/ObstacleAvoidance/ObstacleAvoidance.cpp.o" \
"CMakeFiles/task3.dir/src/Spline/Spline.cpp.o" \
"CMakeFiles/task3.dir/src/Trajectory/LinearTrajectory.cpp.o" \
"CMakeFiles/task3.dir/src/Trajectory/MoveObjectTrajectory.cpp.o" \
"CMakeFiles/task3.dir/src/Trajectory/Trajectory.cpp.o" \
"CMakeFiles/task3.dir/src/UR5/UR5.cpp.o" \
"CMakeFiles/task3.dir/src/tasks/rotations.cpp.o" \
"CMakeFiles/task3.dir/src/utilities/utilities.cpp.o" \
"CMakeFiles/task3.dir/src/visionClient/visionClient.cpp.o"

# External object files for target task3
task3_EXTERNAL_OBJECTS =

/home/leo/UR5-project/devel/lib/robotics_project/task3: CMakeFiles/task3.dir/src/tasks/task3.cpp.o
/home/leo/UR5-project/devel/lib/robotics_project/task3: CMakeFiles/task3.dir/src/Bezier/Bezier.cpp.o
/home/leo/UR5-project/devel/lib/robotics_project/task3: CMakeFiles/task3.dir/src/Bezier/QuinticBezier.cpp.o
/home/leo/UR5-project/devel/lib/robotics_project/task3: CMakeFiles/task3.dir/src/Checkpoint/Checkpoint.cpp.o
/home/leo/UR5-project/devel/lib/robotics_project/task3: CMakeFiles/task3.dir/src/ObstacleAvoidance/Funnel.cpp.o
/home/leo/UR5-project/devel/lib/robotics_project/task3: CMakeFiles/task3.dir/src/ObstacleAvoidance/Hill.cpp.o
/home/leo/UR5-project/devel/lib/robotics_project/task3: CMakeFiles/task3.dir/src/ObstacleAvoidance/Obstacle.cpp.o
/home/leo/UR5-project/devel/lib/robotics_project/task3: CMakeFiles/task3.dir/src/ObstacleAvoidance/ObstacleAvoidance.cpp.o
/home/leo/UR5-project/devel/lib/robotics_project/task3: CMakeFiles/task3.dir/src/Spline/Spline.cpp.o
/home/leo/UR5-project/devel/lib/robotics_project/task3: CMakeFiles/task3.dir/src/Trajectory/LinearTrajectory.cpp.o
/home/leo/UR5-project/devel/lib/robotics_project/task3: CMakeFiles/task3.dir/src/Trajectory/MoveObjectTrajectory.cpp.o
/home/leo/UR5-project/devel/lib/robotics_project/task3: CMakeFiles/task3.dir/src/Trajectory/Trajectory.cpp.o
/home/leo/UR5-project/devel/lib/robotics_project/task3: CMakeFiles/task3.dir/src/UR5/UR5.cpp.o
/home/leo/UR5-project/devel/lib/robotics_project/task3: CMakeFiles/task3.dir/src/tasks/rotations.cpp.o
/home/leo/UR5-project/devel/lib/robotics_project/task3: CMakeFiles/task3.dir/src/utilities/utilities.cpp.o
/home/leo/UR5-project/devel/lib/robotics_project/task3: CMakeFiles/task3.dir/src/visionClient/visionClient.cpp.o
/home/leo/UR5-project/devel/lib/robotics_project/task3: CMakeFiles/task3.dir/build.make
/home/leo/UR5-project/devel/lib/robotics_project/task3: /opt/ros/noetic/lib/libroscpp.so
/home/leo/UR5-project/devel/lib/robotics_project/task3: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/leo/UR5-project/devel/lib/robotics_project/task3: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/leo/UR5-project/devel/lib/robotics_project/task3: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/leo/UR5-project/devel/lib/robotics_project/task3: /opt/ros/noetic/lib/librosconsole.so
/home/leo/UR5-project/devel/lib/robotics_project/task3: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/leo/UR5-project/devel/lib/robotics_project/task3: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/leo/UR5-project/devel/lib/robotics_project/task3: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/leo/UR5-project/devel/lib/robotics_project/task3: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/leo/UR5-project/devel/lib/robotics_project/task3: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/leo/UR5-project/devel/lib/robotics_project/task3: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/leo/UR5-project/devel/lib/robotics_project/task3: /opt/ros/noetic/lib/librostime.so
/home/leo/UR5-project/devel/lib/robotics_project/task3: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/leo/UR5-project/devel/lib/robotics_project/task3: /opt/ros/noetic/lib/libcpp_common.so
/home/leo/UR5-project/devel/lib/robotics_project/task3: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/leo/UR5-project/devel/lib/robotics_project/task3: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/leo/UR5-project/devel/lib/robotics_project/task3: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/leo/UR5-project/devel/lib/robotics_project/task3: CMakeFiles/task3.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/leo/UR5-project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_17) "Linking CXX executable /home/leo/UR5-project/devel/lib/robotics_project/task3"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/task3.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/task3.dir/build: /home/leo/UR5-project/devel/lib/robotics_project/task3

.PHONY : CMakeFiles/task3.dir/build

CMakeFiles/task3.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/task3.dir/cmake_clean.cmake
.PHONY : CMakeFiles/task3.dir/clean

CMakeFiles/task3.dir/depend:
	cd /home/leo/UR5-project/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leo/UR5-project/src /home/leo/UR5-project/src /home/leo/UR5-project/build /home/leo/UR5-project/build /home/leo/UR5-project/build/CMakeFiles/task3.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/task3.dir/depend

