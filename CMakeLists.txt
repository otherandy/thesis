cmake_minimum_required(VERSION 3.1...3.23)
project(ExplorationBot)


# CGAL
find_package(CGAL QUIET COMPONENTS)
if (NOT CGAL_FOUND)
  message(STATUS "This project requires the CGAL library, and will not be compiled.")
  return()
endif()

# Boost
find_package(Boost REQUIRED)
if (NOT Boost_FOUND)
  message(STATUS "This project requires the Boost library, and will not be compiled.")
  return()
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
    GIT_TAG v5.0.1
)
FetchContent_MakeAvailable(raylib_cpp)

# ############################

set(SOURCES ExplorationBot.cpp)

add_executable(${PROJECT_NAME} ${SOURCES})
add_to_cached_list(CGAL_EXECUTABLE_TARGETS ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 11)

target_link_libraries(${PROJECT_NAME} PRIVATE CGAL::CGAL)
target_link_libraries(${PROJECT_NAME} PUBLIC raylib raylib_cpp)
