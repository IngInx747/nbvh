# CMakeLists of libraries to build as static libs.
# The libs are used by multiple executables and other libs.

# glfw
set(GLFW_BUILD_DOCS "Build the GLFW documentation" OFF)
set(GLFW_INSTALL "Generate installation target" OFF)
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/glfw")

# glfw
set(GLFW_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/glfw/include" PARENT_SCOPE)

# glm
set(GLM_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/glm/glm" PARENT_SCOPE)

# stb
set(STB_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/stb" PARENT_SCOPE)
