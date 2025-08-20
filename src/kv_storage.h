#ifndef __KV_STORAGE__

#include "Fusion/FusionMath.h"
#include <string>
#include <vector>

// NOTE: Enum is declared here, but the strings are declared in the cpp-file.

enum {
    cal_mag_offset,
    cal_mag_gain,
    cal_acc_offset,
    cal_acc_gain,
    cal_gyro_offset,
    cal_gyro_gain,
    cal_euler_output_offset
};

// std::string kv_keys[7] = {"MagOffset", "MagGain", "AccOffset", "AccGain",
// "GyroOffset", "GyroGain", "OutputOffset"};

extern std::string kv_keys[7];

bool kv_store_initialized(void);
bool kv_store_save_calibration(const std::string &key,
                               const FusionVector &data);
bool kv_store_load_calibration(const std::string &key, FusionVector &calib,
                               FusionVector &factory_default);

bool kv_store_save_calibration(const std::string &key,
                               const FusionMatrix &data);
bool kv_store_load_calibration(const std::string &key, FusionMatrix &calib,
                               FusionMatrix &factory_default);

void kv_store_reset(std::vector<float> params);

#endif // !f __KV_STORAGE__
