project(ray-tracer)

file(GLOB SRCS
    "*.h"
    "*.hh"
    "*.hpp"
    "*.c"
    "*.cc"
    "*.cpp")

add_executable(${PROJECT_NAME} ${SRCS})
include_directories("${CMAKE_SOURCE_DIR}")
include_directories("${CMAKE_SOURCE_DIR}/3rdparty")

# stb
find_package(stb REQUIRED HINTS "${CMAKE_SOURCE_DIR}/3rdparty/stb")
target_include_directories(${PROJECT_NAME} INTERFACE "${STB_INCLUDE_DIRS}")
include_directories("${STB_INCLUDE_DIRS}")
