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
CMAKE_COMMAND = /home/student/workspace/clion-2017.2.3/bin/cmake/bin/cmake

# The command to remove a file.
RM = /home/student/workspace/clion-2017.2.3/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/student/workspace/RoVi-2017/pluginV1

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/student/workspace/RoVi-2017/pluginV1/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/VisualServoing.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/VisualServoing.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/VisualServoing.dir/flags.make

src/moc_VisualServoing.cpp: ../src/VisualServoing.hpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/student/workspace/RoVi-2017/pluginV1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating src/moc_VisualServoing.cpp"
	cd /home/student/workspace/RoVi-2017/pluginV1/cmake-build-debug/src && /usr/lib/x86_64-linux-gnu/qt5/bin/moc @/home/student/workspace/RoVi-2017/pluginV1/cmake-build-debug/src/moc_VisualServoing.cpp_parameters

src/moc_vs.cpp: ../src/vs.h
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/student/workspace/RoVi-2017/pluginV1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating src/moc_vs.cpp"
	cd /home/student/workspace/RoVi-2017/pluginV1/cmake-build-debug/src && /usr/lib/x86_64-linux-gnu/qt5/bin/moc @/home/student/workspace/RoVi-2017/pluginV1/cmake-build-debug/src/moc_vs.cpp_parameters

src/moc_vsMult.cpp: ../src/vsMult.h
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/student/workspace/RoVi-2017/pluginV1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Generating src/moc_vsMult.cpp"
	cd /home/student/workspace/RoVi-2017/pluginV1/cmake-build-debug/src && /usr/lib/x86_64-linux-gnu/qt5/bin/moc @/home/student/workspace/RoVi-2017/pluginV1/cmake-build-debug/src/moc_vsMult.cpp_parameters

src/moc_ip.cpp: ../src/ip.h
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/student/workspace/RoVi-2017/pluginV1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Generating src/moc_ip.cpp"
	cd /home/student/workspace/RoVi-2017/pluginV1/cmake-build-debug/src && /usr/lib/x86_64-linux-gnu/qt5/bin/moc @/home/student/workspace/RoVi-2017/pluginV1/cmake-build-debug/src/moc_ip.cpp_parameters

qrc_resources.cpp: ../src/pa_icon.png
qrc_resources.cpp: ../markers/Marker1.ppm
qrc_resources.cpp: ../markers/Marker2a.ppm
qrc_resources.cpp: ../markers/Marker3.ppm
qrc_resources.cpp: ../src/resources.qrc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/student/workspace/RoVi-2017/pluginV1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Generating qrc_resources.cpp"
	/usr/lib/x86_64-linux-gnu/qt5/bin/rcc --name resources --output /home/student/workspace/RoVi-2017/pluginV1/cmake-build-debug/qrc_resources.cpp /home/student/workspace/RoVi-2017/pluginV1/src/resources.qrc

CMakeFiles/VisualServoing.dir/src/VisualServoing.cpp.o: CMakeFiles/VisualServoing.dir/flags.make
CMakeFiles/VisualServoing.dir/src/VisualServoing.cpp.o: ../src/VisualServoing.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/student/workspace/RoVi-2017/pluginV1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/VisualServoing.dir/src/VisualServoing.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VisualServoing.dir/src/VisualServoing.cpp.o -c /home/student/workspace/RoVi-2017/pluginV1/src/VisualServoing.cpp

CMakeFiles/VisualServoing.dir/src/VisualServoing.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VisualServoing.dir/src/VisualServoing.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/student/workspace/RoVi-2017/pluginV1/src/VisualServoing.cpp > CMakeFiles/VisualServoing.dir/src/VisualServoing.cpp.i

CMakeFiles/VisualServoing.dir/src/VisualServoing.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VisualServoing.dir/src/VisualServoing.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/student/workspace/RoVi-2017/pluginV1/src/VisualServoing.cpp -o CMakeFiles/VisualServoing.dir/src/VisualServoing.cpp.s

CMakeFiles/VisualServoing.dir/src/VisualServoing.cpp.o.requires:

.PHONY : CMakeFiles/VisualServoing.dir/src/VisualServoing.cpp.o.requires

CMakeFiles/VisualServoing.dir/src/VisualServoing.cpp.o.provides: CMakeFiles/VisualServoing.dir/src/VisualServoing.cpp.o.requires
	$(MAKE) -f CMakeFiles/VisualServoing.dir/build.make CMakeFiles/VisualServoing.dir/src/VisualServoing.cpp.o.provides.build
.PHONY : CMakeFiles/VisualServoing.dir/src/VisualServoing.cpp.o.provides

CMakeFiles/VisualServoing.dir/src/VisualServoing.cpp.o.provides.build: CMakeFiles/VisualServoing.dir/src/VisualServoing.cpp.o


CMakeFiles/VisualServoing.dir/src/vs.cpp.o: CMakeFiles/VisualServoing.dir/flags.make
CMakeFiles/VisualServoing.dir/src/vs.cpp.o: ../src/vs.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/student/workspace/RoVi-2017/pluginV1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/VisualServoing.dir/src/vs.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VisualServoing.dir/src/vs.cpp.o -c /home/student/workspace/RoVi-2017/pluginV1/src/vs.cpp

CMakeFiles/VisualServoing.dir/src/vs.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VisualServoing.dir/src/vs.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/student/workspace/RoVi-2017/pluginV1/src/vs.cpp > CMakeFiles/VisualServoing.dir/src/vs.cpp.i

CMakeFiles/VisualServoing.dir/src/vs.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VisualServoing.dir/src/vs.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/student/workspace/RoVi-2017/pluginV1/src/vs.cpp -o CMakeFiles/VisualServoing.dir/src/vs.cpp.s

CMakeFiles/VisualServoing.dir/src/vs.cpp.o.requires:

.PHONY : CMakeFiles/VisualServoing.dir/src/vs.cpp.o.requires

CMakeFiles/VisualServoing.dir/src/vs.cpp.o.provides: CMakeFiles/VisualServoing.dir/src/vs.cpp.o.requires
	$(MAKE) -f CMakeFiles/VisualServoing.dir/build.make CMakeFiles/VisualServoing.dir/src/vs.cpp.o.provides.build
.PHONY : CMakeFiles/VisualServoing.dir/src/vs.cpp.o.provides

CMakeFiles/VisualServoing.dir/src/vs.cpp.o.provides.build: CMakeFiles/VisualServoing.dir/src/vs.cpp.o


CMakeFiles/VisualServoing.dir/src/vsMult.cpp.o: CMakeFiles/VisualServoing.dir/flags.make
CMakeFiles/VisualServoing.dir/src/vsMult.cpp.o: ../src/vsMult.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/student/workspace/RoVi-2017/pluginV1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/VisualServoing.dir/src/vsMult.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VisualServoing.dir/src/vsMult.cpp.o -c /home/student/workspace/RoVi-2017/pluginV1/src/vsMult.cpp

CMakeFiles/VisualServoing.dir/src/vsMult.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VisualServoing.dir/src/vsMult.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/student/workspace/RoVi-2017/pluginV1/src/vsMult.cpp > CMakeFiles/VisualServoing.dir/src/vsMult.cpp.i

CMakeFiles/VisualServoing.dir/src/vsMult.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VisualServoing.dir/src/vsMult.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/student/workspace/RoVi-2017/pluginV1/src/vsMult.cpp -o CMakeFiles/VisualServoing.dir/src/vsMult.cpp.s

CMakeFiles/VisualServoing.dir/src/vsMult.cpp.o.requires:

.PHONY : CMakeFiles/VisualServoing.dir/src/vsMult.cpp.o.requires

CMakeFiles/VisualServoing.dir/src/vsMult.cpp.o.provides: CMakeFiles/VisualServoing.dir/src/vsMult.cpp.o.requires
	$(MAKE) -f CMakeFiles/VisualServoing.dir/build.make CMakeFiles/VisualServoing.dir/src/vsMult.cpp.o.provides.build
.PHONY : CMakeFiles/VisualServoing.dir/src/vsMult.cpp.o.provides

CMakeFiles/VisualServoing.dir/src/vsMult.cpp.o.provides.build: CMakeFiles/VisualServoing.dir/src/vsMult.cpp.o


CMakeFiles/VisualServoing.dir/src/ip.cpp.o: CMakeFiles/VisualServoing.dir/flags.make
CMakeFiles/VisualServoing.dir/src/ip.cpp.o: ../src/ip.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/student/workspace/RoVi-2017/pluginV1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/VisualServoing.dir/src/ip.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VisualServoing.dir/src/ip.cpp.o -c /home/student/workspace/RoVi-2017/pluginV1/src/ip.cpp

CMakeFiles/VisualServoing.dir/src/ip.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VisualServoing.dir/src/ip.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/student/workspace/RoVi-2017/pluginV1/src/ip.cpp > CMakeFiles/VisualServoing.dir/src/ip.cpp.i

CMakeFiles/VisualServoing.dir/src/ip.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VisualServoing.dir/src/ip.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/student/workspace/RoVi-2017/pluginV1/src/ip.cpp -o CMakeFiles/VisualServoing.dir/src/ip.cpp.s

CMakeFiles/VisualServoing.dir/src/ip.cpp.o.requires:

.PHONY : CMakeFiles/VisualServoing.dir/src/ip.cpp.o.requires

CMakeFiles/VisualServoing.dir/src/ip.cpp.o.provides: CMakeFiles/VisualServoing.dir/src/ip.cpp.o.requires
	$(MAKE) -f CMakeFiles/VisualServoing.dir/build.make CMakeFiles/VisualServoing.dir/src/ip.cpp.o.provides.build
.PHONY : CMakeFiles/VisualServoing.dir/src/ip.cpp.o.provides

CMakeFiles/VisualServoing.dir/src/ip.cpp.o.provides.build: CMakeFiles/VisualServoing.dir/src/ip.cpp.o


CMakeFiles/VisualServoing.dir/src/moc_VisualServoing.cpp.o: CMakeFiles/VisualServoing.dir/flags.make
CMakeFiles/VisualServoing.dir/src/moc_VisualServoing.cpp.o: src/moc_VisualServoing.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/student/workspace/RoVi-2017/pluginV1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object CMakeFiles/VisualServoing.dir/src/moc_VisualServoing.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VisualServoing.dir/src/moc_VisualServoing.cpp.o -c /home/student/workspace/RoVi-2017/pluginV1/cmake-build-debug/src/moc_VisualServoing.cpp

CMakeFiles/VisualServoing.dir/src/moc_VisualServoing.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VisualServoing.dir/src/moc_VisualServoing.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/student/workspace/RoVi-2017/pluginV1/cmake-build-debug/src/moc_VisualServoing.cpp > CMakeFiles/VisualServoing.dir/src/moc_VisualServoing.cpp.i

CMakeFiles/VisualServoing.dir/src/moc_VisualServoing.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VisualServoing.dir/src/moc_VisualServoing.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/student/workspace/RoVi-2017/pluginV1/cmake-build-debug/src/moc_VisualServoing.cpp -o CMakeFiles/VisualServoing.dir/src/moc_VisualServoing.cpp.s

CMakeFiles/VisualServoing.dir/src/moc_VisualServoing.cpp.o.requires:

.PHONY : CMakeFiles/VisualServoing.dir/src/moc_VisualServoing.cpp.o.requires

CMakeFiles/VisualServoing.dir/src/moc_VisualServoing.cpp.o.provides: CMakeFiles/VisualServoing.dir/src/moc_VisualServoing.cpp.o.requires
	$(MAKE) -f CMakeFiles/VisualServoing.dir/build.make CMakeFiles/VisualServoing.dir/src/moc_VisualServoing.cpp.o.provides.build
.PHONY : CMakeFiles/VisualServoing.dir/src/moc_VisualServoing.cpp.o.provides

CMakeFiles/VisualServoing.dir/src/moc_VisualServoing.cpp.o.provides.build: CMakeFiles/VisualServoing.dir/src/moc_VisualServoing.cpp.o


CMakeFiles/VisualServoing.dir/src/moc_vs.cpp.o: CMakeFiles/VisualServoing.dir/flags.make
CMakeFiles/VisualServoing.dir/src/moc_vs.cpp.o: src/moc_vs.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/student/workspace/RoVi-2017/pluginV1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object CMakeFiles/VisualServoing.dir/src/moc_vs.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VisualServoing.dir/src/moc_vs.cpp.o -c /home/student/workspace/RoVi-2017/pluginV1/cmake-build-debug/src/moc_vs.cpp

CMakeFiles/VisualServoing.dir/src/moc_vs.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VisualServoing.dir/src/moc_vs.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/student/workspace/RoVi-2017/pluginV1/cmake-build-debug/src/moc_vs.cpp > CMakeFiles/VisualServoing.dir/src/moc_vs.cpp.i

CMakeFiles/VisualServoing.dir/src/moc_vs.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VisualServoing.dir/src/moc_vs.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/student/workspace/RoVi-2017/pluginV1/cmake-build-debug/src/moc_vs.cpp -o CMakeFiles/VisualServoing.dir/src/moc_vs.cpp.s

CMakeFiles/VisualServoing.dir/src/moc_vs.cpp.o.requires:

.PHONY : CMakeFiles/VisualServoing.dir/src/moc_vs.cpp.o.requires

CMakeFiles/VisualServoing.dir/src/moc_vs.cpp.o.provides: CMakeFiles/VisualServoing.dir/src/moc_vs.cpp.o.requires
	$(MAKE) -f CMakeFiles/VisualServoing.dir/build.make CMakeFiles/VisualServoing.dir/src/moc_vs.cpp.o.provides.build
.PHONY : CMakeFiles/VisualServoing.dir/src/moc_vs.cpp.o.provides

CMakeFiles/VisualServoing.dir/src/moc_vs.cpp.o.provides.build: CMakeFiles/VisualServoing.dir/src/moc_vs.cpp.o


CMakeFiles/VisualServoing.dir/src/moc_vsMult.cpp.o: CMakeFiles/VisualServoing.dir/flags.make
CMakeFiles/VisualServoing.dir/src/moc_vsMult.cpp.o: src/moc_vsMult.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/student/workspace/RoVi-2017/pluginV1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building CXX object CMakeFiles/VisualServoing.dir/src/moc_vsMult.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VisualServoing.dir/src/moc_vsMult.cpp.o -c /home/student/workspace/RoVi-2017/pluginV1/cmake-build-debug/src/moc_vsMult.cpp

CMakeFiles/VisualServoing.dir/src/moc_vsMult.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VisualServoing.dir/src/moc_vsMult.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/student/workspace/RoVi-2017/pluginV1/cmake-build-debug/src/moc_vsMult.cpp > CMakeFiles/VisualServoing.dir/src/moc_vsMult.cpp.i

CMakeFiles/VisualServoing.dir/src/moc_vsMult.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VisualServoing.dir/src/moc_vsMult.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/student/workspace/RoVi-2017/pluginV1/cmake-build-debug/src/moc_vsMult.cpp -o CMakeFiles/VisualServoing.dir/src/moc_vsMult.cpp.s

CMakeFiles/VisualServoing.dir/src/moc_vsMult.cpp.o.requires:

.PHONY : CMakeFiles/VisualServoing.dir/src/moc_vsMult.cpp.o.requires

CMakeFiles/VisualServoing.dir/src/moc_vsMult.cpp.o.provides: CMakeFiles/VisualServoing.dir/src/moc_vsMult.cpp.o.requires
	$(MAKE) -f CMakeFiles/VisualServoing.dir/build.make CMakeFiles/VisualServoing.dir/src/moc_vsMult.cpp.o.provides.build
.PHONY : CMakeFiles/VisualServoing.dir/src/moc_vsMult.cpp.o.provides

CMakeFiles/VisualServoing.dir/src/moc_vsMult.cpp.o.provides.build: CMakeFiles/VisualServoing.dir/src/moc_vsMult.cpp.o


CMakeFiles/VisualServoing.dir/src/moc_ip.cpp.o: CMakeFiles/VisualServoing.dir/flags.make
CMakeFiles/VisualServoing.dir/src/moc_ip.cpp.o: src/moc_ip.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/student/workspace/RoVi-2017/pluginV1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Building CXX object CMakeFiles/VisualServoing.dir/src/moc_ip.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VisualServoing.dir/src/moc_ip.cpp.o -c /home/student/workspace/RoVi-2017/pluginV1/cmake-build-debug/src/moc_ip.cpp

CMakeFiles/VisualServoing.dir/src/moc_ip.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VisualServoing.dir/src/moc_ip.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/student/workspace/RoVi-2017/pluginV1/cmake-build-debug/src/moc_ip.cpp > CMakeFiles/VisualServoing.dir/src/moc_ip.cpp.i

CMakeFiles/VisualServoing.dir/src/moc_ip.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VisualServoing.dir/src/moc_ip.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/student/workspace/RoVi-2017/pluginV1/cmake-build-debug/src/moc_ip.cpp -o CMakeFiles/VisualServoing.dir/src/moc_ip.cpp.s

CMakeFiles/VisualServoing.dir/src/moc_ip.cpp.o.requires:

.PHONY : CMakeFiles/VisualServoing.dir/src/moc_ip.cpp.o.requires

CMakeFiles/VisualServoing.dir/src/moc_ip.cpp.o.provides: CMakeFiles/VisualServoing.dir/src/moc_ip.cpp.o.requires
	$(MAKE) -f CMakeFiles/VisualServoing.dir/build.make CMakeFiles/VisualServoing.dir/src/moc_ip.cpp.o.provides.build
.PHONY : CMakeFiles/VisualServoing.dir/src/moc_ip.cpp.o.provides

CMakeFiles/VisualServoing.dir/src/moc_ip.cpp.o.provides.build: CMakeFiles/VisualServoing.dir/src/moc_ip.cpp.o


CMakeFiles/VisualServoing.dir/qrc_resources.cpp.o: CMakeFiles/VisualServoing.dir/flags.make
CMakeFiles/VisualServoing.dir/qrc_resources.cpp.o: qrc_resources.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/student/workspace/RoVi-2017/pluginV1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Building CXX object CMakeFiles/VisualServoing.dir/qrc_resources.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VisualServoing.dir/qrc_resources.cpp.o -c /home/student/workspace/RoVi-2017/pluginV1/cmake-build-debug/qrc_resources.cpp

CMakeFiles/VisualServoing.dir/qrc_resources.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VisualServoing.dir/qrc_resources.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/student/workspace/RoVi-2017/pluginV1/cmake-build-debug/qrc_resources.cpp > CMakeFiles/VisualServoing.dir/qrc_resources.cpp.i

CMakeFiles/VisualServoing.dir/qrc_resources.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VisualServoing.dir/qrc_resources.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/student/workspace/RoVi-2017/pluginV1/cmake-build-debug/qrc_resources.cpp -o CMakeFiles/VisualServoing.dir/qrc_resources.cpp.s

CMakeFiles/VisualServoing.dir/qrc_resources.cpp.o.requires:

.PHONY : CMakeFiles/VisualServoing.dir/qrc_resources.cpp.o.requires

CMakeFiles/VisualServoing.dir/qrc_resources.cpp.o.provides: CMakeFiles/VisualServoing.dir/qrc_resources.cpp.o.requires
	$(MAKE) -f CMakeFiles/VisualServoing.dir/build.make CMakeFiles/VisualServoing.dir/qrc_resources.cpp.o.provides.build
.PHONY : CMakeFiles/VisualServoing.dir/qrc_resources.cpp.o.provides

CMakeFiles/VisualServoing.dir/qrc_resources.cpp.o.provides.build: CMakeFiles/VisualServoing.dir/qrc_resources.cpp.o


# Object files for target VisualServoing
VisualServoing_OBJECTS = \
"CMakeFiles/VisualServoing.dir/src/VisualServoing.cpp.o" \
"CMakeFiles/VisualServoing.dir/src/vs.cpp.o" \
"CMakeFiles/VisualServoing.dir/src/vsMult.cpp.o" \
"CMakeFiles/VisualServoing.dir/src/ip.cpp.o" \
"CMakeFiles/VisualServoing.dir/src/moc_VisualServoing.cpp.o" \
"CMakeFiles/VisualServoing.dir/src/moc_vs.cpp.o" \
"CMakeFiles/VisualServoing.dir/src/moc_vsMult.cpp.o" \
"CMakeFiles/VisualServoing.dir/src/moc_ip.cpp.o" \
"CMakeFiles/VisualServoing.dir/qrc_resources.cpp.o"

# External object files for target VisualServoing
VisualServoing_EXTERNAL_OBJECTS =

../libs/Debug/libVisualServoing.so: CMakeFiles/VisualServoing.dir/src/VisualServoing.cpp.o
../libs/Debug/libVisualServoing.so: CMakeFiles/VisualServoing.dir/src/vs.cpp.o
../libs/Debug/libVisualServoing.so: CMakeFiles/VisualServoing.dir/src/vsMult.cpp.o
../libs/Debug/libVisualServoing.so: CMakeFiles/VisualServoing.dir/src/ip.cpp.o
../libs/Debug/libVisualServoing.so: CMakeFiles/VisualServoing.dir/src/moc_VisualServoing.cpp.o
../libs/Debug/libVisualServoing.so: CMakeFiles/VisualServoing.dir/src/moc_vs.cpp.o
../libs/Debug/libVisualServoing.so: CMakeFiles/VisualServoing.dir/src/moc_vsMult.cpp.o
../libs/Debug/libVisualServoing.so: CMakeFiles/VisualServoing.dir/src/moc_ip.cpp.o
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
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/student/workspace/RoVi-2017/pluginV1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_15) "Linking CXX shared module ../libs/Debug/libVisualServoing.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/VisualServoing.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/VisualServoing.dir/build: ../libs/Debug/libVisualServoing.so

.PHONY : CMakeFiles/VisualServoing.dir/build

CMakeFiles/VisualServoing.dir/requires: CMakeFiles/VisualServoing.dir/src/VisualServoing.cpp.o.requires
CMakeFiles/VisualServoing.dir/requires: CMakeFiles/VisualServoing.dir/src/vs.cpp.o.requires
CMakeFiles/VisualServoing.dir/requires: CMakeFiles/VisualServoing.dir/src/vsMult.cpp.o.requires
CMakeFiles/VisualServoing.dir/requires: CMakeFiles/VisualServoing.dir/src/ip.cpp.o.requires
CMakeFiles/VisualServoing.dir/requires: CMakeFiles/VisualServoing.dir/src/moc_VisualServoing.cpp.o.requires
CMakeFiles/VisualServoing.dir/requires: CMakeFiles/VisualServoing.dir/src/moc_vs.cpp.o.requires
CMakeFiles/VisualServoing.dir/requires: CMakeFiles/VisualServoing.dir/src/moc_vsMult.cpp.o.requires
CMakeFiles/VisualServoing.dir/requires: CMakeFiles/VisualServoing.dir/src/moc_ip.cpp.o.requires
CMakeFiles/VisualServoing.dir/requires: CMakeFiles/VisualServoing.dir/qrc_resources.cpp.o.requires

.PHONY : CMakeFiles/VisualServoing.dir/requires

CMakeFiles/VisualServoing.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/VisualServoing.dir/cmake_clean.cmake
.PHONY : CMakeFiles/VisualServoing.dir/clean

CMakeFiles/VisualServoing.dir/depend: src/moc_VisualServoing.cpp
CMakeFiles/VisualServoing.dir/depend: src/moc_vs.cpp
CMakeFiles/VisualServoing.dir/depend: src/moc_vsMult.cpp
CMakeFiles/VisualServoing.dir/depend: src/moc_ip.cpp
CMakeFiles/VisualServoing.dir/depend: qrc_resources.cpp
	cd /home/student/workspace/RoVi-2017/pluginV1/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/student/workspace/RoVi-2017/pluginV1 /home/student/workspace/RoVi-2017/pluginV1 /home/student/workspace/RoVi-2017/pluginV1/cmake-build-debug /home/student/workspace/RoVi-2017/pluginV1/cmake-build-debug /home/student/workspace/RoVi-2017/pluginV1/cmake-build-debug/CMakeFiles/VisualServoing.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/VisualServoing.dir/depend

