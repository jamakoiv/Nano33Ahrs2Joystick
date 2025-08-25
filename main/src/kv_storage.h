#ifndef __KV_STORAGE__

#include "Fusion/FusionMath.h"
#include <string>

using std::string;

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

extern string kv_keys[7];

bool kv_store_initialized(void);
bool kv_store_save_calibration(const string key, const FusionVector &data);
bool kv_store_load_calibration(const string key, FusionVector &calib,
                               FusionVector &factory_default);

bool kv_store_save_calibration(const string key, const FusionMatrix &data);
bool kv_store_load_calibration(const string key, FusionMatrix &calib,
                               FusionMatrix &factory_default);

void kv_store_reset(void);

#endif // !f __KV_STORAGE__
