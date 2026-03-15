cmake_minimum_required(VERSION 3.16)
project(ExplorationBot LANGUAGES CXX)

# CMake >= 3.30 removed FindBoost
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
  set(CMAKE_POLICY_DEFAULT_CMP0167 NEW)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(CGAL REQUIRED)
find_package(Boost REQUIRED)

if(TARGET Boost::headers)
  set(BOOST_TARGET Boost::headers)
else()
  set(BOOST_TARGET Boost::boost)
endif()

include(FetchContent)

# Configure fetched dependencies for this project.
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
  raylib
  GIT_REPOSITORY https://github.com/raysan5/raylib.git
  GIT_TAG 5.0
)

FetchContent_Declare(
  raylib_cpp
  GIT_REPOSITORY https://github.com/RobLoach/raylib-cpp.git
  GIT_TAG v5.0.3
)

FetchContent_MakeAvailable(raylib raylib_cpp)

# CONFIGURE_DEPENDS keeps file globs in sync when files are added/removed.
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
file(GLOB_RECURSE HEADERS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp")

add_executable(${PROJECT_NAME})

target_sources(
  ${PROJECT_NAME}
  PRIVATE
    ${SOURCES}
    ${HEADERS}
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/constants
    ${CMAKE_CURRENT_SOURCE_DIR}/include/types
)

target_link_libraries(
  ${PROJECT_NAME}
  PRIVATE
    CGAL::CGAL
    ${BOOST_TARGET}
    raylib
    raylib_cpp
)

# Register target for CGAL helper tooling.
add_to_cached_list(CGAL_EXECUTABLE_TARGETS ${PROJECT_NAME})
