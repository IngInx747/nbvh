# CMakeLists of libraries to build as static libs.
# The libs are used by multiple executables and other libs.

# glfw
add_subdirectory("${CMAKE_SOURCE_DIR}/3rdparty/glfw")
set(GLFW_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/3rdparty/glfw/include")
