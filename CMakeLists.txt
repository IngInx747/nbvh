cmake_minimum_required(VERSION 3.5.0)
project(NBVH VERSION 1.0 LANGUAGES CXX)

string(COMPARE EQUAL "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_SOURCE_DIR}" NBVH_STANDALONE)

if (NBVH_STANDALONE)
    add_subdirectory(test/test_aabb)
    add_subdirectory(test/test_bvh)
endif()
