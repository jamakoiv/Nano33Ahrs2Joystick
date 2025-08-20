#include "Fusion/FusionAhrs.h"
#include "Fusion/FusionMath.h"

/*
 * Global variables from main file.
 */

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

extern FusionVector AxisOffset;
extern FusionVector AxisOffset_default;

extern FusionAhrs AHRS;
extern float CompassHeading;
extern int SerialOutputMode;
