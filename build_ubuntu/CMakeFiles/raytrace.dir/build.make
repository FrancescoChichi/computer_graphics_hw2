# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/francesco/Documents/computer_graphics_hw2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/francesco/Documents/computer_graphics_hw2/build_ubuntu

# Include any dependencies generated for this target.
include CMakeFiles/raytrace.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/raytrace.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/raytrace.dir/flags.make

CMakeFiles/raytrace.dir/src/raytrace.cpp.o: CMakeFiles/raytrace.dir/flags.make
CMakeFiles/raytrace.dir/src/raytrace.cpp.o: ../src/raytrace.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/francesco/Documents/computer_graphics_hw2/build_ubuntu/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/raytrace.dir/src/raytrace.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/raytrace.dir/src/raytrace.cpp.o -c /home/francesco/Documents/computer_graphics_hw2/src/raytrace.cpp

CMakeFiles/raytrace.dir/src/raytrace.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/raytrace.dir/src/raytrace.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/francesco/Documents/computer_graphics_hw2/src/raytrace.cpp > CMakeFiles/raytrace.dir/src/raytrace.cpp.i

CMakeFiles/raytrace.dir/src/raytrace.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/raytrace.dir/src/raytrace.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/francesco/Documents/computer_graphics_hw2/src/raytrace.cpp -o CMakeFiles/raytrace.dir/src/raytrace.cpp.s

CMakeFiles/raytrace.dir/src/raytrace.cpp.o.requires:

.PHONY : CMakeFiles/raytrace.dir/src/raytrace.cpp.o.requires

CMakeFiles/raytrace.dir/src/raytrace.cpp.o.provides: CMakeFiles/raytrace.dir/src/raytrace.cpp.o.requires
	$(MAKE) -f CMakeFiles/raytrace.dir/build.make CMakeFiles/raytrace.dir/src/raytrace.cpp.o.provides.build
.PHONY : CMakeFiles/raytrace.dir/src/raytrace.cpp.o.provides

CMakeFiles/raytrace.dir/src/raytrace.cpp.o.provides.build: CMakeFiles/raytrace.dir/src/raytrace.cpp.o


# Object files for target raytrace
raytrace_OBJECTS = \
"CMakeFiles/raytrace.dir/src/raytrace.cpp.o"

# External object files for target raytrace
raytrace_EXTERNAL_OBJECTS =

../bin/raytrace: CMakeFiles/raytrace.dir/src/raytrace.cpp.o
../bin/raytrace: CMakeFiles/raytrace.dir/build.make
../bin/raytrace: ../bin/libhwlib.a
../bin/raytrace: CMakeFiles/raytrace.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/francesco/Documents/computer_graphics_hw2/build_ubuntu/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../bin/raytrace"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/raytrace.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/raytrace.dir/build: ../bin/raytrace

.PHONY : CMakeFiles/raytrace.dir/build

CMakeFiles/raytrace.dir/requires: CMakeFiles/raytrace.dir/src/raytrace.cpp.o.requires

.PHONY : CMakeFiles/raytrace.dir/requires

CMakeFiles/raytrace.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/raytrace.dir/cmake_clean.cmake
.PHONY : CMakeFiles/raytrace.dir/clean

CMakeFiles/raytrace.dir/depend:
	cd /home/francesco/Documents/computer_graphics_hw2/build_ubuntu && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/francesco/Documents/computer_graphics_hw2 /home/francesco/Documents/computer_graphics_hw2 /home/francesco/Documents/computer_graphics_hw2/build_ubuntu /home/francesco/Documents/computer_graphics_hw2/build_ubuntu /home/francesco/Documents/computer_graphics_hw2/build_ubuntu/CMakeFiles/raytrace.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/raytrace.dir/depend

