#include "utils.h"
#include <sstream>

std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> res;
    std::stringstream ss(s);
    std::string item;

    while (std::getline(ss, item, delim)) {
        res.push_back(item);
    }

    return res;
}

std::vector<std::string> split_between(const std::string &s, char start_char,
                                       char stop_char) {
    std::vector<std::string> res;

    int pos0 = 0;
    int pos1 = 0;
    int last_pos0 = 0;

    while (true) {
        pos0 = s.find(start_char, pos1);
        pos1 = s.find(stop_char, pos0);

        if (pos0 == std::string::npos) {
            break;
        }

        std::string substr = s.substr(pos0, pos1 - pos0 + 1);
        res.push_back(substr);
    }

    return res;
}

void set_calib_helper(const std::vector<float> &data,
                      MyVector::vector &offset) {
    if (data.size() < 3) {
        Serial.println("Invalid input: Could not parse 3 floats from input.");
    } else {
        offset.x = data[0];
        offset.y = data[1];
        offset.z = data[2];
    }
}

void set_calib_helper(const std::vector<float> &data, MyVector::vector &offset,
                      MyVector::vector &gain) {
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

float remap_yaw(float yaw, float d) {
    float overlap = 0;
    float res = yaw + d;

    if (res > 180) {
        overlap = res - 180;
        res = -180 + overlap;

    } else if (res <= -180) {
        overlap = res + 180;
        res = 180 + overlap;
    }

    return res;
}
