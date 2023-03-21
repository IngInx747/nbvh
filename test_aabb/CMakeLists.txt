project(test-aabb)

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
