# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.25

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

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = "C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"

# The command to remove a file.
RM = "C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = C:\Users\kamil\Desktop\tp_projekt_3\tp_projekt_3

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = C:\Users\kamil\Desktop\tp_projekt_3\build

# Include any dependencies generated for this target.
include SDL/CMakeFiles/SDL2main.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include SDL/CMakeFiles/SDL2main.dir/compiler_depend.make

# Include the progress variables for this target.
include SDL/CMakeFiles/SDL2main.dir/progress.make

# Include the compile flags for this target's objects.
include SDL/CMakeFiles/SDL2main.dir/flags.make

SDL/CMakeFiles/SDL2main.dir/src/main/windows/SDL_windows_main.c.obj: SDL/CMakeFiles/SDL2main.dir/flags.make
SDL/CMakeFiles/SDL2main.dir/src/main/windows/SDL_windows_main.c.obj: SDL/CMakeFiles/SDL2main.dir/includes_C.rsp
SDL/CMakeFiles/SDL2main.dir/src/main/windows/SDL_windows_main.c.obj: C:/Users/kamil/Desktop/tp_projekt_3/tp_projekt_3/SDL/src/main/windows/SDL_windows_main.c
SDL/CMakeFiles/SDL2main.dir/src/main/windows/SDL_windows_main.c.obj: SDL/CMakeFiles/SDL2main.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\kamil\Desktop\tp_projekt_3\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object SDL/CMakeFiles/SDL2main.dir/src/main/windows/SDL_windows_main.c.obj"
	cd /d C:\Users\kamil\Desktop\tp_projekt_3\build\SDL && C:\msys64\mingw64\bin\gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT SDL/CMakeFiles/SDL2main.dir/src/main/windows/SDL_windows_main.c.obj -MF CMakeFiles\SDL2main.dir\src\main\windows\SDL_windows_main.c.obj.d -o CMakeFiles\SDL2main.dir\src\main\windows\SDL_windows_main.c.obj -c C:\Users\kamil\Desktop\tp_projekt_3\tp_projekt_3\SDL\src\main\windows\SDL_windows_main.c

SDL/CMakeFiles/SDL2main.dir/src/main/windows/SDL_windows_main.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/SDL2main.dir/src/main/windows/SDL_windows_main.c.i"
	cd /d C:\Users\kamil\Desktop\tp_projekt_3\build\SDL && C:\msys64\mingw64\bin\gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E C:\Users\kamil\Desktop\tp_projekt_3\tp_projekt_3\SDL\src\main\windows\SDL_windows_main.c > CMakeFiles\SDL2main.dir\src\main\windows\SDL_windows_main.c.i

SDL/CMakeFiles/SDL2main.dir/src/main/windows/SDL_windows_main.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/SDL2main.dir/src/main/windows/SDL_windows_main.c.s"
	cd /d C:\Users\kamil\Desktop\tp_projekt_3\build\SDL && C:\msys64\mingw64\bin\gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S C:\Users\kamil\Desktop\tp_projekt_3\tp_projekt_3\SDL\src\main\windows\SDL_windows_main.c -o CMakeFiles\SDL2main.dir\src\main\windows\SDL_windows_main.c.s

# Object files for target SDL2main
SDL2main_OBJECTS = \
"CMakeFiles/SDL2main.dir/src/main/windows/SDL_windows_main.c.obj"

# External object files for target SDL2main
SDL2main_EXTERNAL_OBJECTS =

SDL/libSDL2maind.a: SDL/CMakeFiles/SDL2main.dir/src/main/windows/SDL_windows_main.c.obj
SDL/libSDL2maind.a: SDL/CMakeFiles/SDL2main.dir/build.make
SDL/libSDL2maind.a: SDL/CMakeFiles/SDL2main.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=C:\Users\kamil\Desktop\tp_projekt_3\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C static library libSDL2maind.a"
	cd /d C:\Users\kamil\Desktop\tp_projekt_3\build\SDL && $(CMAKE_COMMAND) -P CMakeFiles\SDL2main.dir\cmake_clean_target.cmake
	cd /d C:\Users\kamil\Desktop\tp_projekt_3\build\SDL && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\SDL2main.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
SDL/CMakeFiles/SDL2main.dir/build: SDL/libSDL2maind.a
.PHONY : SDL/CMakeFiles/SDL2main.dir/build

SDL/CMakeFiles/SDL2main.dir/clean:
	cd /d C:\Users\kamil\Desktop\tp_projekt_3\build\SDL && $(CMAKE_COMMAND) -P CMakeFiles\SDL2main.dir\cmake_clean.cmake
.PHONY : SDL/CMakeFiles/SDL2main.dir/clean

SDL/CMakeFiles/SDL2main.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" C:\Users\kamil\Desktop\tp_projekt_3\tp_projekt_3 C:\Users\kamil\Desktop\tp_projekt_3\tp_projekt_3\SDL C:\Users\kamil\Desktop\tp_projekt_3\build C:\Users\kamil\Desktop\tp_projekt_3\build\SDL C:\Users\kamil\Desktop\tp_projekt_3\build\SDL\CMakeFiles\SDL2main.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : SDL/CMakeFiles/SDL2main.dir/depend

