cmake_minimum_required(VERSION 3.16)
project(ExplorationBot LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# CGAL
find_package(CGAL REQUIRED COMPONENTS)
if (NOT CGAL_FOUND)
  message(FATAL_ERROR "This project requires the CGAL library.")
endif()

# Boost
find_package(Boost REQUIRED)
if (NOT Boost_FOUND)
  message(FATAL_ERROR "This project requires the Boost library.")
endif()

# raylib
include(FetchContent)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG 5.0
)
FetchContent_MakeAvailable(raylib)

# raylib-cpp
FetchContent_Declare(
    raylib_cpp
    GIT_REPOSITORY https://github.com/RobLoach/raylib-cpp.git
    GIT_TAG v5.0.3
)
FetchContent_MakeAvailable(raylib_cpp)

# Source files
file(GLOB_RECURSE SOURCES "src/*.cpp")

# Executable
add_executable(${PROJECT_NAME} ${SOURCES})
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 17)

# Link libraries
target_link_libraries(${PROJECT_NAME} PRIVATE CGAL::CGAL Boost::boost)
target_link_libraries(${PROJECT_NAME} PUBLIC raylib raylib_cpp)

# Register with CGAL
add_to_cached_list(CGAL_EXECUTABLE_TARGETS ${PROJECT_NAME})
