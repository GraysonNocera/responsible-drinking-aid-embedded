# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.26

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/brian/.local/lib/python3.10/site-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /home/brian/.local/lib/python3.10/site-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/brian/Documents/ECE477/responsible-drinking-aid-embedded

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/brian/Documents/ECE477/responsible-drinking-aid-embedded/build

# Include any dependencies generated for this target.
include CMakeFiles/responsible-drinking-aid-embedded.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/responsible-drinking-aid-embedded.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/responsible-drinking-aid-embedded.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/responsible-drinking-aid-embedded.dir/flags.make

CMakeFiles/responsible-drinking-aid-embedded.dir/src/main.cpp.o: CMakeFiles/responsible-drinking-aid-embedded.dir/flags.make
CMakeFiles/responsible-drinking-aid-embedded.dir/src/main.cpp.o: /home/brian/Documents/ECE477/responsible-drinking-aid-embedded/src/main.cpp
CMakeFiles/responsible-drinking-aid-embedded.dir/src/main.cpp.o: CMakeFiles/responsible-drinking-aid-embedded.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/brian/Documents/ECE477/responsible-drinking-aid-embedded/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/responsible-drinking-aid-embedded.dir/src/main.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/responsible-drinking-aid-embedded.dir/src/main.cpp.o -MF CMakeFiles/responsible-drinking-aid-embedded.dir/src/main.cpp.o.d -o CMakeFiles/responsible-drinking-aid-embedded.dir/src/main.cpp.o -c /home/brian/Documents/ECE477/responsible-drinking-aid-embedded/src/main.cpp

CMakeFiles/responsible-drinking-aid-embedded.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/responsible-drinking-aid-embedded.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/brian/Documents/ECE477/responsible-drinking-aid-embedded/src/main.cpp > CMakeFiles/responsible-drinking-aid-embedded.dir/src/main.cpp.i

CMakeFiles/responsible-drinking-aid-embedded.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/responsible-drinking-aid-embedded.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/brian/Documents/ECE477/responsible-drinking-aid-embedded/src/main.cpp -o CMakeFiles/responsible-drinking-aid-embedded.dir/src/main.cpp.s

# Object files for target responsible-drinking-aid-embedded
responsible__drinking__aid__embedded_OBJECTS = \
"CMakeFiles/responsible-drinking-aid-embedded.dir/src/main.cpp.o"

# External object files for target responsible-drinking-aid-embedded
responsible__drinking__aid__embedded_EXTERNAL_OBJECTS =

responsible-drinking-aid-embedded: CMakeFiles/responsible-drinking-aid-embedded.dir/src/main.cpp.o
responsible-drinking-aid-embedded: CMakeFiles/responsible-drinking-aid-embedded.dir/build.make
responsible-drinking-aid-embedded: CMakeFiles/responsible-drinking-aid-embedded.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/brian/Documents/ECE477/responsible-drinking-aid-embedded/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable responsible-drinking-aid-embedded"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/responsible-drinking-aid-embedded.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/responsible-drinking-aid-embedded.dir/build: responsible-drinking-aid-embedded
.PHONY : CMakeFiles/responsible-drinking-aid-embedded.dir/build

CMakeFiles/responsible-drinking-aid-embedded.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/responsible-drinking-aid-embedded.dir/cmake_clean.cmake
.PHONY : CMakeFiles/responsible-drinking-aid-embedded.dir/clean

CMakeFiles/responsible-drinking-aid-embedded.dir/depend:
	cd /home/brian/Documents/ECE477/responsible-drinking-aid-embedded/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/brian/Documents/ECE477/responsible-drinking-aid-embedded /home/brian/Documents/ECE477/responsible-drinking-aid-embedded /home/brian/Documents/ECE477/responsible-drinking-aid-embedded/build /home/brian/Documents/ECE477/responsible-drinking-aid-embedded/build /home/brian/Documents/ECE477/responsible-drinking-aid-embedded/build/CMakeFiles/responsible-drinking-aid-embedded.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/responsible-drinking-aid-embedded.dir/depend

