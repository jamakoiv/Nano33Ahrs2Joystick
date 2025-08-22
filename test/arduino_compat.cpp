
#include "arduino_compat.h"
#include <iostream>

void Serial::println(char *msg) { std::cout << msg << std::endl; }

void Serial::println(int n, int format) { std::cout << n << std::endl; }

void Serial::println(float n, int format) { std::cout << n << std::endl; }
