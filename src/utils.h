
#ifndef __STRING_HELPERS__

#include <string>
#include <vector>

#include "MyVector/MyVector.h"

using MyVector::vector;

std::vector<std::string> split_input(std::string input,
                                     const std::string &delimiter);
std::vector<float> split_and_strtof(std::string input,
                                    const std::string &delimiter);

void set_calib_helper(const std::vector<float> &data, vector &offset);
void set_calib_helper(const std::vector<float> &data, vector &offset,
                      vector &gain);

float remap_yaw(float yaw, float d);

#endif // __STRING_HELPERS
