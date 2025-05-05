
#ifndef __UTILS__

#include <string>
#include <vector>

#include "MyVector/MyVector.h"

std::vector<std::string> split_input(std::string input,
                                     const std::string &delimiter);
std::vector<float> split_and_strtof(std::string input,
                                    const std::string &delimiter);

void set_calib_helper(const std::vector<float> &data, MyVector::vector &offset);
void set_calib_helper(const std::vector<float> &data, MyVector::vector &offset,
                      MyVector::vector &gain);

float remap_yaw(float yaw, float d);

#endif // __UTILS__
