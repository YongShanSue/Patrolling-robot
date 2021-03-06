# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/robotvision/code/assistive-robotics-repo/kinect-lcm

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/robotvision/code/assistive-robotics-repo/kinect-lcm/pod-build

# Include any dependencies generated for this target.
include src/kinect-utils/CMakeFiles/kinect-utils.dir/depend.make

# Include the progress variables for this target.
include src/kinect-utils/CMakeFiles/kinect-utils.dir/progress.make

# Include the compile flags for this target's objects.
include src/kinect-utils/CMakeFiles/kinect-utils.dir/flags.make

src/kinect-utils/CMakeFiles/kinect-utils.dir/kinect-calib.c.o: src/kinect-utils/CMakeFiles/kinect-utils.dir/flags.make
src/kinect-utils/CMakeFiles/kinect-utils.dir/kinect-calib.c.o: ../src/kinect-utils/kinect-calib.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/robotvision/code/assistive-robotics-repo/kinect-lcm/pod-build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object src/kinect-utils/CMakeFiles/kinect-utils.dir/kinect-calib.c.o"
	cd /home/robotvision/code/assistive-robotics-repo/kinect-lcm/pod-build/src/kinect-utils && /usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/kinect-utils.dir/kinect-calib.c.o   -c /home/robotvision/code/assistive-robotics-repo/kinect-lcm/src/kinect-utils/kinect-calib.c

src/kinect-utils/CMakeFiles/kinect-utils.dir/kinect-calib.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/kinect-utils.dir/kinect-calib.c.i"
	cd /home/robotvision/code/assistive-robotics-repo/kinect-lcm/pod-build/src/kinect-utils && /usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -E /home/robotvision/code/assistive-robotics-repo/kinect-lcm/src/kinect-utils/kinect-calib.c > CMakeFiles/kinect-utils.dir/kinect-calib.c.i

src/kinect-utils/CMakeFiles/kinect-utils.dir/kinect-calib.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/kinect-utils.dir/kinect-calib.c.s"
	cd /home/robotvision/code/assistive-robotics-repo/kinect-lcm/pod-build/src/kinect-utils && /usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -S /home/robotvision/code/assistive-robotics-repo/kinect-lcm/src/kinect-utils/kinect-calib.c -o CMakeFiles/kinect-utils.dir/kinect-calib.c.s

src/kinect-utils/CMakeFiles/kinect-utils.dir/kinect-calib.c.o.requires:
.PHONY : src/kinect-utils/CMakeFiles/kinect-utils.dir/kinect-calib.c.o.requires

src/kinect-utils/CMakeFiles/kinect-utils.dir/kinect-calib.c.o.provides: src/kinect-utils/CMakeFiles/kinect-utils.dir/kinect-calib.c.o.requires
	$(MAKE) -f src/kinect-utils/CMakeFiles/kinect-utils.dir/build.make src/kinect-utils/CMakeFiles/kinect-utils.dir/kinect-calib.c.o.provides.build
.PHONY : src/kinect-utils/CMakeFiles/kinect-utils.dir/kinect-calib.c.o.provides

src/kinect-utils/CMakeFiles/kinect-utils.dir/kinect-calib.c.o.provides.build: src/kinect-utils/CMakeFiles/kinect-utils.dir/kinect-calib.c.o

# Object files for target kinect-utils
kinect__utils_OBJECTS = \
"CMakeFiles/kinect-utils.dir/kinect-calib.c.o"

# External object files for target kinect-utils
kinect__utils_EXTERNAL_OBJECTS =

lib/libkinect-utils.so.1: src/kinect-utils/CMakeFiles/kinect-utils.dir/kinect-calib.c.o
lib/libkinect-utils.so.1: src/kinect-utils/CMakeFiles/kinect-utils.dir/build.make
lib/libkinect-utils.so.1: src/kinect-utils/CMakeFiles/kinect-utils.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking C shared library ../../lib/libkinect-utils.so"
	cd /home/robotvision/code/assistive-robotics-repo/kinect-lcm/pod-build/src/kinect-utils && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/kinect-utils.dir/link.txt --verbose=$(VERBOSE)
	cd /home/robotvision/code/assistive-robotics-repo/kinect-lcm/pod-build/src/kinect-utils && $(CMAKE_COMMAND) -E cmake_symlink_library ../../lib/libkinect-utils.so.1 ../../lib/libkinect-utils.so.1 ../../lib/libkinect-utils.so

lib/libkinect-utils.so: lib/libkinect-utils.so.1

# Rule to build all files generated by this target.
src/kinect-utils/CMakeFiles/kinect-utils.dir/build: lib/libkinect-utils.so
.PHONY : src/kinect-utils/CMakeFiles/kinect-utils.dir/build

src/kinect-utils/CMakeFiles/kinect-utils.dir/requires: src/kinect-utils/CMakeFiles/kinect-utils.dir/kinect-calib.c.o.requires
.PHONY : src/kinect-utils/CMakeFiles/kinect-utils.dir/requires

src/kinect-utils/CMakeFiles/kinect-utils.dir/clean:
	cd /home/robotvision/code/assistive-robotics-repo/kinect-lcm/pod-build/src/kinect-utils && $(CMAKE_COMMAND) -P CMakeFiles/kinect-utils.dir/cmake_clean.cmake
.PHONY : src/kinect-utils/CMakeFiles/kinect-utils.dir/clean

src/kinect-utils/CMakeFiles/kinect-utils.dir/depend:
	cd /home/robotvision/code/assistive-robotics-repo/kinect-lcm/pod-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robotvision/code/assistive-robotics-repo/kinect-lcm /home/robotvision/code/assistive-robotics-repo/kinect-lcm/src/kinect-utils /home/robotvision/code/assistive-robotics-repo/kinect-lcm/pod-build /home/robotvision/code/assistive-robotics-repo/kinect-lcm/pod-build/src/kinect-utils /home/robotvision/code/assistive-robotics-repo/kinect-lcm/pod-build/src/kinect-utils/CMakeFiles/kinect-utils.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/kinect-utils/CMakeFiles/kinect-utils.dir/depend

