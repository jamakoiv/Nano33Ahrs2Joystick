cmake_minimum_required(VERSION 3.30)
project(tests)

add_library(arduino_compat ../main/src/arduino_compat.cpp)

add_library(utils ../main/src/utils.cpp)
add_library(test_utils test_utils.cpp)
link_libraries(test_utils utils)

add_library(serial_comms ../main/src/serial_comms.cpp)
add_library(test_serial_comms test_serial_comms.cpp)
link_libraries(test_serial_comms serial_comms)

add_executable(tests tests.cpp)
link_libraries(tests test_utils)
