# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.8

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
CMAKE_COMMAND = /home/student/Desktop/workspace/clion-2017.2.3/bin/cmake/bin/cmake

# The command to remove a file.
RM = /home/student/Desktop/workspace/clion-2017.2.3/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/student/Desktop/workspace/Rovi

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/student/Desktop/workspace/Rovi/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/VisualServoing.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/VisualServoing.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/VisualServoing.dir/flags.make

src/moc_VisualServoing.cpp: ../src/VisualServoing.hpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/student/Desktop/workspace/Rovi/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating src/moc_VisualServoing.cpp"
	cd /home/student/Desktop/workspace/Rovi/cmake-build-debug/src && /usr/lib/x86_64-linux-gnu/qt5/bin/moc @/home/student/Desktop/workspace/Rovi/cmake-build-debug/src/moc_VisualServoing.cpp_parameters

qrc_resources.cpp: ../src/pa_icon.png
qrc_resources.cpp: ../src/resources.qrc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/student/Desktop/workspace/Rovi/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating qrc_resources.cpp"
	/usr/lib/x86_64-linux-gnu/qt5/bin/rcc --name resources --output /home/student/Desktop/workspace/Rovi/cmake-build-debug/qrc_resources.cpp /home/student/Desktop/workspace/Rovi/src/resources.qrc

CMakeFiles/VisualServoing.dir/src/VisualServoing.cpp.o: CMakeFiles/VisualServoing.dir/flags.make
CMakeFiles/VisualServoing.dir/src/VisualServoing.cpp.o: ../src/VisualServoing.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/student/Desktop/workspace/Rovi/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/VisualServoing.dir/src/VisualServoing.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VisualServoing.dir/src/VisualServoing.cpp.o -c /home/student/Desktop/workspace/Rovi/src/VisualServoing.cpp

CMakeFiles/VisualServoing.dir/src/VisualServoing.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VisualServoing.dir/src/VisualServoing.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/student/Desktop/workspace/Rovi/src/VisualServoing.cpp > CMakeFiles/VisualServoing.dir/src/VisualServoing.cpp.i

CMakeFiles/VisualServoing.dir/src/VisualServoing.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VisualServoing.dir/src/VisualServoing.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/student/Desktop/workspace/Rovi/src/VisualServoing.cpp -o CMakeFiles/VisualServoing.dir/src/VisualServoing.cpp.s

CMakeFiles/VisualServoing.dir/src/VisualServoing.cpp.o.requires:

.PHONY : CMakeFiles/VisualServoing.dir/src/VisualServoing.cpp.o.requires

CMakeFiles/VisualServoing.dir/src/VisualServoing.cpp.o.provides: CMakeFiles/VisualServoing.dir/src/VisualServoing.cpp.o.requires
	$(MAKE) -f CMakeFiles/VisualServoing.dir/build.make CMakeFiles/VisualServoing.dir/src/VisualServoing.cpp.o.provides.build
.PHONY : CMakeFiles/VisualServoing.dir/src/VisualServoing.cpp.o.provides

CMakeFiles/VisualServoing.dir/src/VisualServoing.cpp.o.provides.build: CMakeFiles/VisualServoing.dir/src/VisualServoing.cpp.o


CMakeFiles/VisualServoing.dir/src/moc_VisualServoing.cpp.o: CMakeFiles/VisualServoing.dir/flags.make
CMakeFiles/VisualServoing.dir/src/moc_VisualServoing.cpp.o: src/moc_VisualServoing.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/student/Desktop/workspace/Rovi/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/VisualServoing.dir/src/moc_VisualServoing.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VisualServoing.dir/src/moc_VisualServoing.cpp.o -c /home/student/Desktop/workspace/Rovi/cmake-build-debug/src/moc_VisualServoing.cpp

CMakeFiles/VisualServoing.dir/src/moc_VisualServoing.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VisualServoing.dir/src/moc_VisualServoing.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/student/Desktop/workspace/Rovi/cmake-build-debug/src/moc_VisualServoing.cpp > CMakeFiles/VisualServoing.dir/src/moc_VisualServoing.cpp.i

CMakeFiles/VisualServoing.dir/src/moc_VisualServoing.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VisualServoing.dir/src/moc_VisualServoing.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/student/Desktop/workspace/Rovi/cmake-build-debug/src/moc_VisualServoing.cpp -o CMakeFiles/VisualServoing.dir/src/moc_VisualServoing.cpp.s

CMakeFiles/VisualServoing.dir/src/moc_VisualServoing.cpp.o.requires:

.PHONY : CMakeFiles/VisualServoing.dir/src/moc_VisualServoing.cpp.o.requires

CMakeFiles/VisualServoing.dir/src/moc_VisualServoing.cpp.o.provides: CMakeFiles/VisualServoing.dir/src/moc_VisualServoing.cpp.o.requires
	$(MAKE) -f CMakeFiles/VisualServoing.dir/build.make CMakeFiles/VisualServoing.dir/src/moc_VisualServoing.cpp.o.provides.build
.PHONY : CMakeFiles/VisualServoing.dir/src/moc_VisualServoing.cpp.o.provides

CMakeFiles/VisualServoing.dir/src/moc_VisualServoing.cpp.o.provides.build: CMakeFiles/VisualServoing.dir/src/moc_VisualServoing.cpp.o


CMakeFiles/VisualServoing.dir/qrc_resources.cpp.o: CMakeFiles/VisualServoing.dir/flags.make
CMakeFiles/VisualServoing.dir/qrc_resources.cpp.o: qrc_resources.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/student/Desktop/workspace/Rovi/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/VisualServoing.dir/qrc_resources.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VisualServoing.dir/qrc_resources.cpp.o -c /home/student/Desktop/workspace/Rovi/cmake-build-debug/qrc_resources.cpp

CMakeFiles/VisualServoing.dir/qrc_resources.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VisualServoing.dir/qrc_resources.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/student/Desktop/workspace/Rovi/cmake-build-debug/qrc_resources.cpp > CMakeFiles/VisualServoing.dir/qrc_resources.cpp.i

CMakeFiles/VisualServoing.dir/qrc_resources.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VisualServoing.dir/qrc_resources.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/student/Desktop/workspace/Rovi/cmake-build-debug/qrc_resources.cpp -o CMakeFiles/VisualServoing.dir/qrc_resources.cpp.s

CMakeFiles/VisualServoing.dir/qrc_resources.cpp.o.requires:

.PHONY : CMakeFiles/VisualServoing.dir/qrc_resources.cpp.o.requires

CMakeFiles/VisualServoing.dir/qrc_resources.cpp.o.provides: CMakeFiles/VisualServoing.dir/qrc_resources.cpp.o.requires
	$(MAKE) -f CMakeFiles/VisualServoing.dir/build.make CMakeFiles/VisualServoing.dir/qrc_resources.cpp.o.provides.build
.PHONY : CMakeFiles/VisualServoing.dir/qrc_resources.cpp.o.provides

CMakeFiles/VisualServoing.dir/qrc_resources.cpp.o.provides.build: CMakeFiles/VisualServoing.dir/qrc_resources.cpp.o


# Object files for target VisualServoing
VisualServoing_OBJECTS = \
"CMakeFiles/VisualServoing.dir/src/VisualServoing.cpp.o" \
"CMakeFiles/VisualServoing.dir/src/moc_VisualServoing.cpp.o" \
"CMakeFiles/VisualServoing.dir/qrc_resources.cpp.o"

# External object files for target VisualServoing
VisualServoing_EXTERNAL_OBJECTS =

../libs/Debug/libVisualServoing.so: CMakeFiles/VisualServoing.dir/src/VisualServoing.cpp.o
../libs/Debug/libVisualServoing.so: CMakeFiles/VisualServoing.dir/src/moc_VisualServoing.cpp.o
../libs/Debug/libVisualServoing.so: CMakeFiles/VisualServoing.dir/qrc_resources.cpp.o
../libs/Debug/libVisualServoing.so: CMakeFiles/VisualServoing.dir/build.make
../libs/Debug/libVisualServoing.so: /usr/lib/x86_64-linux-gnu/libGLU.so
../libs/Debug/libVisualServoing.so: /usr/lib/x86_64-linux-gnu/libGL.so
../libs/Debug/libVisualServoing.so: /usr/lib/x86_64-linux-gnu/libxerces-c.so
../libs/Debug/libVisualServoing.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../libs/Debug/libVisualServoing.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
../libs/Debug/libVisualServoing.so: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
../libs/Debug/libVisualServoing.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
../libs/Debug/libVisualServoing.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
../libs/Debug/libVisualServoing.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
../libs/Debug/libVisualServoing.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
../libs/Debug/libVisualServoing.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../libs/Debug/libVisualServoing.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
../libs/Debug/libVisualServoing.so: /usr/lib/x86_64-linux-gnu/libpthread.so
../libs/Debug/libVisualServoing.so: /usr/lib/x86_64-linux-gnu/libboost_test_exec_monitor.a
../libs/Debug/libVisualServoing.so: /usr/lib/x86_64-linux-gnu/libboost_unit_test_framework.so
../libs/Debug/libVisualServoing.so: /usr/lib/x86_64-linux-gnu/libz.so
../libs/Debug/libVisualServoing.so: /usr/lib/x86_64-linux-gnu/libdl.so
../libs/Debug/libVisualServoing.so: /usr/lib/x86_64-linux-gnu/libQt5OpenGL.so.5.5.1
../libs/Debug/libVisualServoing.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
../libs/Debug/libVisualServoing.so: /usr/lib/x86_64-linux-gnu/libGLU.so
../libs/Debug/libVisualServoing.so: /usr/lib/x86_64-linux-gnu/libGL.so
../libs/Debug/libVisualServoing.so: /usr/lib/x86_64-linux-gnu/libGLU.so
../libs/Debug/libVisualServoing.so: /usr/lib/x86_64-linux-gnu/libGL.so
../libs/Debug/libVisualServoing.so: /usr/lib/x86_64-linux-gnu/libxerces-c.so
../libs/Debug/libVisualServoing.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../libs/Debug/libVisualServoing.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
../libs/Debug/libVisualServoing.so: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
../libs/Debug/libVisualServoing.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
../libs/Debug/libVisualServoing.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
../libs/Debug/libVisualServoing.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
../libs/Debug/libVisualServoing.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
../libs/Debug/libVisualServoing.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../libs/Debug/libVisualServoing.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
../libs/Debug/libVisualServoing.so: /usr/lib/x86_64-linux-gnu/libpthread.so
../libs/Debug/libVisualServoing.so: /usr/lib/x86_64-linux-gnu/libboost_test_exec_monitor.a
../libs/Debug/libVisualServoing.so: /usr/lib/x86_64-linux-gnu/libboost_unit_test_framework.so
../libs/Debug/libVisualServoing.so: /usr/lib/x86_64-linux-gnu/libz.so
../libs/Debug/libVisualServoing.so: /usr/lib/x86_64-linux-gnu/libdl.so
../libs/Debug/libVisualServoing.so: /opt/ros/kinetic/lib/libopencv_stitching3.so.3.2.0
../libs/Debug/libVisualServoing.so: /opt/ros/kinetic/lib/libopencv_superres3.so.3.2.0
../libs/Debug/libVisualServoing.so: /opt/ros/kinetic/lib/libopencv_videostab3.so.3.2.0
../libs/Debug/libVisualServoing.so: /opt/ros/kinetic/lib/libopencv_aruco3.so.3.2.0
../libs/Debug/libVisualServoing.so: /opt/ros/kinetic/lib/libopencv_bgsegm3.so.3.2.0
../libs/Debug/libVisualServoing.so: /opt/ros/kinetic/lib/libopencv_bioinspired3.so.3.2.0
../libs/Debug/libVisualServoing.so: /opt/ros/kinetic/lib/libopencv_ccalib3.so.3.2.0
../libs/Debug/libVisualServoing.so: /opt/ros/kinetic/lib/libopencv_cvv3.so.3.2.0
../libs/Debug/libVisualServoing.so: /opt/ros/kinetic/lib/libopencv_datasets3.so.3.2.0
../libs/Debug/libVisualServoing.so: /opt/ros/kinetic/lib/libopencv_dpm3.so.3.2.0
../libs/Debug/libVisualServoing.so: /opt/ros/kinetic/lib/libopencv_face3.so.3.2.0
../libs/Debug/libVisualServoing.so: /opt/ros/kinetic/lib/libopencv_fuzzy3.so.3.2.0
../libs/Debug/libVisualServoing.so: /opt/ros/kinetic/lib/libopencv_hdf3.so.3.2.0
../libs/Debug/libVisualServoing.so: /opt/ros/kinetic/lib/libopencv_line_descriptor3.so.3.2.0
../libs/Debug/libVisualServoing.so: /opt/ros/kinetic/lib/libopencv_optflow3.so.3.2.0
../libs/Debug/libVisualServoing.so: /opt/ros/kinetic/lib/libopencv_plot3.so.3.2.0
../libs/Debug/libVisualServoing.so: /opt/ros/kinetic/lib/libopencv_reg3.so.3.2.0
../libs/Debug/libVisualServoing.so: /opt/ros/kinetic/lib/libopencv_saliency3.so.3.2.0
../libs/Debug/libVisualServoing.so: /opt/ros/kinetic/lib/libopencv_stereo3.so.3.2.0
../libs/Debug/libVisualServoing.so: /opt/ros/kinetic/lib/libopencv_structured_light3.so.3.2.0
../libs/Debug/libVisualServoing.so: /opt/ros/kinetic/lib/libopencv_surface_matching3.so.3.2.0
../libs/Debug/libVisualServoing.so: /opt/ros/kinetic/lib/libopencv_text3.so.3.2.0
../libs/Debug/libVisualServoing.so: /opt/ros/kinetic/lib/libopencv_xfeatures2d3.so.3.2.0
../libs/Debug/libVisualServoing.so: /opt/ros/kinetic/lib/libopencv_ximgproc3.so.3.2.0
../libs/Debug/libVisualServoing.so: /opt/ros/kinetic/lib/libopencv_xobjdetect3.so.3.2.0
../libs/Debug/libVisualServoing.so: /opt/ros/kinetic/lib/libopencv_xphoto3.so.3.2.0
../libs/Debug/libVisualServoing.so: /opt/ros/kinetic/lib/libopencv_shape3.so.3.2.0
../libs/Debug/libVisualServoing.so: /opt/ros/kinetic/lib/libopencv_video3.so.3.2.0
../libs/Debug/libVisualServoing.so: /opt/ros/kinetic/lib/libopencv_viz3.so.3.2.0
../libs/Debug/libVisualServoing.so: /opt/ros/kinetic/lib/libopencv_phase_unwrapping3.so.3.2.0
../libs/Debug/libVisualServoing.so: /opt/ros/kinetic/lib/libopencv_rgbd3.so.3.2.0
../libs/Debug/libVisualServoing.so: /opt/ros/kinetic/lib/libopencv_calib3d3.so.3.2.0
../libs/Debug/libVisualServoing.so: /opt/ros/kinetic/lib/libopencv_features2d3.so.3.2.0
../libs/Debug/libVisualServoing.so: /opt/ros/kinetic/lib/libopencv_flann3.so.3.2.0
../libs/Debug/libVisualServoing.so: /opt/ros/kinetic/lib/libopencv_objdetect3.so.3.2.0
../libs/Debug/libVisualServoing.so: /opt/ros/kinetic/lib/libopencv_ml3.so.3.2.0
../libs/Debug/libVisualServoing.so: /opt/ros/kinetic/lib/libopencv_highgui3.so.3.2.0
../libs/Debug/libVisualServoing.so: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.5.1
../libs/Debug/libVisualServoing.so: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.5.1
../libs/Debug/libVisualServoing.so: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.5.1
../libs/Debug/libVisualServoing.so: /opt/ros/kinetic/lib/libopencv_photo3.so.3.2.0
../libs/Debug/libVisualServoing.so: /opt/ros/kinetic/lib/libopencv_videoio3.so.3.2.0
../libs/Debug/libVisualServoing.so: /opt/ros/kinetic/lib/libopencv_imgcodecs3.so.3.2.0
../libs/Debug/libVisualServoing.so: /opt/ros/kinetic/lib/libopencv_imgproc3.so.3.2.0
../libs/Debug/libVisualServoing.so: /opt/ros/kinetic/lib/libopencv_core3.so.3.2.0
../libs/Debug/libVisualServoing.so: CMakeFiles/VisualServoing.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/student/Desktop/workspace/Rovi/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX shared module ../libs/Debug/libVisualServoing.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/VisualServoing.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/VisualServoing.dir/build: ../libs/Debug/libVisualServoing.so

.PHONY : CMakeFiles/VisualServoing.dir/build

CMakeFiles/VisualServoing.dir/requires: CMakeFiles/VisualServoing.dir/src/VisualServoing.cpp.o.requires
CMakeFiles/VisualServoing.dir/requires: CMakeFiles/VisualServoing.dir/src/moc_VisualServoing.cpp.o.requires
CMakeFiles/VisualServoing.dir/requires: CMakeFiles/VisualServoing.dir/qrc_resources.cpp.o.requires

.PHONY : CMakeFiles/VisualServoing.dir/requires

CMakeFiles/VisualServoing.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/VisualServoing.dir/cmake_clean.cmake
.PHONY : CMakeFiles/VisualServoing.dir/clean

CMakeFiles/VisualServoing.dir/depend: src/moc_VisualServoing.cpp
CMakeFiles/VisualServoing.dir/depend: qrc_resources.cpp
	cd /home/student/Desktop/workspace/Rovi/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/student/Desktop/workspace/Rovi /home/student/Desktop/workspace/Rovi /home/student/Desktop/workspace/Rovi/cmake-build-debug /home/student/Desktop/workspace/Rovi/cmake-build-debug /home/student/Desktop/workspace/Rovi/cmake-build-debug/CMakeFiles/VisualServoing.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/VisualServoing.dir/depend

