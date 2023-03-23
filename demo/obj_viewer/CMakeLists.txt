project(obj-viewer)

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

# glm
find_package(glm REQUIRED HINTS "${CMAKE_SOURCE_DIR}/3rdparty/glm")
target_include_directories(${PROJECT_NAME} INTERFACE "${GLM_INCLUDE_DIRS}")

# glfw
set(GLFW_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/3rdparty/glfw/include")
target_include_directories(${PROJECT_NAME} PUBLIC "${GLFW_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} glfw)
add_dependencies(${PROJECT_NAME} glfw)

# opengl (calls opengl directly, instead of using a modern extension loader library)
find_package(OpenGL REQUIRED)
target_link_libraries(${PROJECT_NAME} OpenGL::GL)
target_link_libraries(${PROJECT_NAME} OpenGL::GLU)
