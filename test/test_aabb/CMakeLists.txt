file(GLOB SRCS "*.h" "*.hh" "*.hpp" "*.c" "*.cc" "*.cpp")

add_executable(test-aabb ${SRCS})

target_link_libraries(test-aabb PRIVATE ${PROJECT_NAME})