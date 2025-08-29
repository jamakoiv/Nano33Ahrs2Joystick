#include "Fusion/FusionAhrs.h"
#include "Fusion/FusionMath.h"

/*
 * Global variables from main ino-file.
 */

// TODO: The existance of this file is a massive code smell. Try to get rid of
// it eventually.

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
extern FusionMatrix acc_misalignment;

extern FusionVector gyro_raw;
extern FusionVector gyro_calibrated;
extern FusionVector gyro_gain;
extern FusionVector gyro_gain_default;
extern FusionVector gyro_offset;
extern FusionVector gyro_offset_default;
extern FusionMatrix gyro_misalignment;

extern FusionVector AxisOffset;
extern FusionVector AxisOffset_default;

extern FusionAhrs AHRS;
extern float CompassHeading;
extern int SerialOutputMode;
