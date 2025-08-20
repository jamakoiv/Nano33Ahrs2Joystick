#include "Fusion/FusionAhrs.h"
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

extern FusionVector mag_raw;
extern FusionVector mag_calibrated;
extern FusionMatrix soft_iron;
extern FusionMatrix soft_iron_default;
extern FusionVector hard_iron;
extern FusionVector hard_iron_default;

extern FusionVector acc_raw;
extern FusionVector acc_calibrated;
extern FusionVector acc_gain;
extern FusionVector acc_gain_default;
extern FusionVector acc_offset;
extern FusionVector acc_offset_default;

extern FusionVector gyro_raw;
extern FusionVector gyro_calibrated;
extern FusionVector gyro_gain;
extern FusionVector gyro_gain_default;
extern FusionVector gyro_offset;
extern FusionVector gyro_offset_default;

extern vector AxisOffset;
extern vector AxisOffset_default;

extern FusionAhrs AHRS;
extern float CompassHeading;
extern int SerialOutputMode;
