
#ifndef __UTILS__

#include <string>
#include <vector>

#include "MyVector/MyVector.h"

std::vector<std::string> split(const std::string &s, char delim);
std::vector<std::string> split_between(const std::string &s, char start_char,
                                       char stop_char);

void set_calib_helper(const std::vector<float> &data, MyVector::vector &offset);
void set_calib_helper(const std::vector<float> &data, MyVector::vector &offset,
                      MyVector::vector &gain);

float remap_yaw(float yaw, float d);

#endif // __UTILS__
