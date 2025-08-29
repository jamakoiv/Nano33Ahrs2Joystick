cmake_minimum_required(VERSION 3.30)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
project(tests)

add_library(utils ../main/src/utils.cpp)
add_executable(test_utils test_utils.cpp)
target_link_libraries(test_utils PRIVATE utils)

add_library(serial_utils ../main/src/serial_utils.cpp)
add_executable(test_serial_utils test_serial_utils.cpp)
target_link_libraries(test_serial_utils PRIVATE serial_utils)
