file(GLOB SRCS "*.hh" "*.cc")
add_executable(gen-box ${SRCS})
target_link_libraries(gen-box PRIVATE ${PROJECT_NAME})