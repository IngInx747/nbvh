file(GLOB SRCS "*.hh" "*.cc")
add_executable(test-bvh ${SRCS})
target_link_libraries(test-bvh PRIVATE ${PROJECT_NAME})