cmake_minimum_required(VERSION 4.0)
project(tests)

add_library(utils ../utils.cpp)
add_library(test_utils test_utils.cpp)
link_libraries(test_utils utils)

add_executable(tests tests.cpp)
link_libraries(tests test_utils)
