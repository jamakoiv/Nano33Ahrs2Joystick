#include <Serial.h>
#include <string>
#include <vector>

#include "MyVector/MyVector.h"

using MyVector::vector;

std::vector<std::string> split_input(std::string input,
                                     const std::string &delimiter) {
  /*
    Split string at 'delimiter' and return the pieces in a
    std::vector<std::string>.
  */
  std::vector<std::string> res;

  auto pos = input.find(delimiter);

  while (pos != std::string::npos) {
    res.emplace_back(input.substr(0, pos));
    input.erase(0, pos + 1);
    pos = input.find(delimiter);
  }

  // Handle the last element if there is no delimiter at the end of the input
  // string.
  if (input.empty()) {
    return res;
  } else {
    res.emplace_back(input);
    return res;
  }
}

std::vector<float> split_and_strtof(std::string input,
                                    const std::string &delimiter) {
  /*
    Split string, convert elements to float and return them as
    std::vector<float>.
  */
  std::vector<float> res;
  auto input_str_vec = split_input(input, delimiter);
  for (std::string str : input_str_vec) {
    res.emplace_back(strtof(str.c_str(), NULL));
  }

  return res;
}

void set_calib_helper(const std::vector<float> &data, vector &offset,
                      vector &gain) {
  /*

  */
  if (data.size() < 6) {
    Serial.println("Invalid input: Could not parse 6 floats from input.");
  } else {
    offset.x = data[0];
    offset.y = data[1];
    offset.z = data[2];
    gain.x = data[3];
    gain.y = data[4];
    gain.z = data[5];
  }
}
