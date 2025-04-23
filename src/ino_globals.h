#include "MyVector/MyVector.h"

using MyVector::vector;

/*
 * Global variables from main file.
 */

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
