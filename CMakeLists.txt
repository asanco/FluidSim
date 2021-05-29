cmake_minimum_required(VERSION 3.13)

project(UnitedEngine)

# --------- Global compiler options -------------
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# THIS IS IN CASE YOU WANT TO SET THE BUILD TO HAVE THE DEBUG SYMBOLS AND SO ON.
# The best way is to enable this when generating the CMake files by running
# cmake -DCMAKE_BUILD_TYPE=Debug <PATH-TO-THE-CMakeLists.txt>
option(CMAKE_BUILD_TYPE "Type of build, by default is Debug" Debug)

#---Pre compiled libraries
find_package(SFML COMPONENTS graphics window system REQUIRED)
if(SFML_FOUND)
	message(STATUS "SFML found!")
	message(STATUS "SFML_LIBRARIES is set to ${SFML_LIBRARIES}")
	message(STATUS "SFML_INCLUDE_DIRS is set to ${SFML_INCLUDE_DIRS}")
else()
	message(ERROR "SFML not found!")
endif()
FIND_PACKAGE(Threads REQUIRED)

# ----------------- Sources -----------------------
# This adds the subdirectories to "load" the other CMakeLists.txt.
add_subdirectory(lib)
add_subdirectory(src)