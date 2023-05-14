file(GLOB SRCS "*.h" "*.hh" "*.hpp" "*.c" "*.cc" "*.cpp")

add_executable(test-bvh ${SRCS})

target_link_libraries(test-bvh PRIVATE ${PROJECT_NAME})