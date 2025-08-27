#ifndef __UTILS__

#include "Fusion/FusionMath.h"
#include <string>
#include <vector>

std::string int_to_hex(int i);

std::vector<std::string> split(const std::string &s, char delim);
std::vector<std::string> split_between(const std::string &s, char start_char,
                                       char stop_char);
std::vector<std::string> split_input(std::vector<float> params,
                                     const std::string &delimiter);
std::vector<float> split_and_strtof(std::string input,
                                    const std::string &delimiter);

float remap_yaw(float yaw, float d);

#endif // __UTILS__
