project(test-aabb)
file(GLOB SRCS "*.h" "*.hh" "*.hpp" "*.c" "*.cc" "*.cpp")
add_executable(${PROJECT_NAME} ${SRCS})
target_include_directories(${PROJECT_NAME} PRIVATE "${CMAKE_SOURCE_DIR}")