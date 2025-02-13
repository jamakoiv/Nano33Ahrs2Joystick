#ifndef __KV_STORAGE__

#include "MyVector/MyVector.h"

#include <string>

/*
 * Global variables from main file.
 */

using MyVector::vector;

extern vector Acc;
extern vector rawAcc;
extern vector CurrentAcc;
extern vector AccGain;
extern vector AccOffset;
extern vector AccGain_default;
extern vector AccOffset_default;

// Variables for gyroscope values and calibrations.
extern vector Gyro;
extern vector rawGyro;
extern vector CurrentGyro;
extern vector GyroGain;
extern vector GyroOffset;
extern vector GyroGain_default;
extern vector GyroOffset_default;

// Variables for magnetometer values and calibrations.
extern vector Mag;
extern vector rawMag;
extern vector CurrentMag;
extern vector MagGain;
extern vector MagOffset;
extern vector MagGain_default;
extern vector MagOffset_default;

bool kv_store_initialized(void);
bool kv_store_save_calibration(const std::string &key, const vector &data);
bool kv_store_load_calibration(const std::string &key, vector &calib,
                               vector &factory_default);
void kv_store_reset(std::string input);

#endif // !f __KV_STORAGE__
