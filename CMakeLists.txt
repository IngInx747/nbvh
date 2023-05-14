cmake_minimum_required(VERSION 3.0)

project(nbvh VERSION 1.0 LANGUAGES CXX)

add_subdirectory(nbvh)

if (${CMAKE_CURRENT_SOURCE_DIR} STREQUAL ${CMAKE_SOURCE_DIR})
    add_subdirectory(test/test_aabb)
    add_subdirectory(test/test_bvh)
endif()
