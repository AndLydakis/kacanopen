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
CMAKE_SOURCE_DIR = /home/lydakis/kacanopen/inno_can

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lydakis/kacanopen/inno_can

# Include any dependencies generated for this target.
include CMakeFiles/./ticks_test.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/./ticks_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/./ticks_test.dir/flags.make

CMakeFiles/./ticks_test.dir/src/ticks_test.cpp.o: CMakeFiles/./ticks_test.dir/flags.make
CMakeFiles/./ticks_test.dir/src/ticks_test.cpp.o: src/ticks_test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lydakis/kacanopen/inno_can/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/./ticks_test.dir/src/ticks_test.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/./ticks_test.dir/src/ticks_test.cpp.o -c /home/lydakis/kacanopen/inno_can/src/ticks_test.cpp

CMakeFiles/./ticks_test.dir/src/ticks_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/./ticks_test.dir/src/ticks_test.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lydakis/kacanopen/inno_can/src/ticks_test.cpp > CMakeFiles/./ticks_test.dir/src/ticks_test.cpp.i

CMakeFiles/./ticks_test.dir/src/ticks_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/./ticks_test.dir/src/ticks_test.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lydakis/kacanopen/inno_can/src/ticks_test.cpp -o CMakeFiles/./ticks_test.dir/src/ticks_test.cpp.s

CMakeFiles/./ticks_test.dir/src/ticks_test.cpp.o.requires:

.PHONY : CMakeFiles/./ticks_test.dir/src/ticks_test.cpp.o.requires

CMakeFiles/./ticks_test.dir/src/ticks_test.cpp.o.provides: CMakeFiles/./ticks_test.dir/src/ticks_test.cpp.o.requires
	$(MAKE) -f CMakeFiles/./ticks_test.dir/build.make CMakeFiles/./ticks_test.dir/src/ticks_test.cpp.o.provides.build
.PHONY : CMakeFiles/./ticks_test.dir/src/ticks_test.cpp.o.provides

CMakeFiles/./ticks_test.dir/src/ticks_test.cpp.o.provides.build: CMakeFiles/./ticks_test.dir/src/ticks_test.cpp.o


# Object files for target ./ticks_test
_/ticks_test_OBJECTS = \
"CMakeFiles/./ticks_test.dir/src/ticks_test.cpp.o"

# External object files for target ./ticks_test
_/ticks_test_EXTERNAL_OBJECTS =

./ticks_test: CMakeFiles/./ticks_test.dir/src/ticks_test.cpp.o
./ticks_test: CMakeFiles/./ticks_test.dir/build.make
./ticks_test: CMakeFiles/./ticks_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lydakis/kacanopen/inno_can/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ./ticks_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/./ticks_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/./ticks_test.dir/build: ./ticks_test

.PHONY : CMakeFiles/./ticks_test.dir/build

CMakeFiles/./ticks_test.dir/requires: CMakeFiles/./ticks_test.dir/src/ticks_test.cpp.o.requires

.PHONY : CMakeFiles/./ticks_test.dir/requires

CMakeFiles/./ticks_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/./ticks_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/./ticks_test.dir/clean

CMakeFiles/./ticks_test.dir/depend:
	cd /home/lydakis/kacanopen/inno_can && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lydakis/kacanopen/inno_can /home/lydakis/kacanopen/inno_can /home/lydakis/kacanopen/inno_can /home/lydakis/kacanopen/inno_can /home/lydakis/kacanopen/inno_can/CMakeFiles/ticks_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/./ticks_test.dir/depend

