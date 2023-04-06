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

# stb
target_include_directories(${PROJECT_NAME} PRIVATE "${STB_INCLUDE_DIRS}")
