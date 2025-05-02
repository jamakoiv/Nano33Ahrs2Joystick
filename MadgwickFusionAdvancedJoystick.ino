#include <map>
#include <string>
#include <vector>
#include <cstring>

#include "serial_commands.h"
#include "src/Fusion/Fusion.h"
#include "src/USBJoystick/USBCommsJoystick.h"
#include "src/MyVector/MyVector.h"
#include "src/LSM9DS1/LSM9DS1.h"

#include "src/serial_comms.h"
#include "src/kv_storage.h"
#include "src/string_helpers.h"

/*
 * TODO: Using MyVector::vector and std::vector together is confusing 
 *     and asking for trouble.
 * TODO: Maybe try having less global variables...
 * TODO: Axis ranges are defined in multiple places with magic numbers. Replace with
 *      central definition.
*/

USBCommsJoystick usb_comms;
Joystick joystick;

int SerialOutputMode = SERIAL_PRINT_AHRS;

/*
 IMU measurement variables and calibration.
*/
using MyVector::vector;

/*
  Fusion-library objects and variables.
*/
const uint32_t SAMPLE_RATE = 238;
FusionOffset AHRS_gyro_offset;
FusionAhrs AHRS;
float CompassHeading;

// Set AHRS algorithm settings
const FusionAhrsSettings AHRSsettings = {
        .gain = 0.50f,
        .accelerationRejection = 10.0f,
        .magneticRejection = 20.0f,
        .rejectionTimeout = 5*SAMPLE_RATE
        //.rejectionTimeout = 0
} ;

// Variables for acceleration values and calibrations.
vector Acc; 
vector rawAcc;
vector CurrentAcc;
vector AccGain;
vector AccOffset;
vector AccGain_default(1.00f, 1.00f, 1.00f);
vector AccOffset_default(0.0325f, 0.0306f, 0.01819f); 

// Variables for gyroscope values and calibrations.
vector Gyro;
vector rawGyro;
vector CurrentGyro;
vector GyroGain;
vector GyroOffset;
vector GyroGain_default(1.125f, 1.125f, 1.125f); 
vector GyroOffset_default(-0.50019f, -0.68556f, 0.13808f);     // 238 Hz values

// Variables for magnetometer values and calibrations.
vector Mag;
vector rawMag;
vector CurrentMag;
vector MagGain;
vector MagOffset;
vector MagGain_default(1.0f, 1.0f, 1.0f);
vector MagOffset_default(-7.257f, 39.747f, -11.817f);

/* AxisOffset(yaw, pitch, roll). Add constant offset to the AHRS-Euler 
 * output as degrees. Generally you should leave these zero and mitigate 
 * output default position problems in the OS software side.
 * BUG: Due to MyVector::vector these must still be accessed as AxisOffset.x etc.
 */
vector AxisOffset;
vector AxisOffset_default(90.0f, 0.0f, 0.0f);

const float GYRO_INTEG = 0.60f;
const float ACC_INTEG = 0.60f;
const float MAG_INTEG = 0.60f;

FusionVector readAcceleration(vector& rawAcc) {
    /* Read and smooth the IMU-data. Uses leaky integrator smoothing. */
    IMU.readAcceleration( rawAcc.x, rawAcc.y, rawAcc.z );
    Acc = (rawAcc + AccOffset) * AccGain;
    Acc = changeAxisSign(Acc, -1, -1, 1);
    CurrentAcc = CurrentAcc*ACC_INTEG + Acc*(1-ACC_INTEG);

    return MyVector_to_FusionVector( CurrentAcc );
}

FusionVector readGyroscope(vector& rawGyro) {
    IMU.readGyroscope( rawGyro.x, rawGyro.y, rawGyro.z );
    Gyro = (rawGyro + GyroOffset) * GyroGain;
    Gyro = changeAxisSign( Gyro, 1, 1, -1 );
    CurrentGyro = CurrentGyro*GYRO_INTEG + Gyro*(1-GYRO_INTEG);

    return MyVector_to_FusionVector( CurrentGyro );
}

FusionVector readMagneticField(vector& rawMag) {
    IMU.readMagneticField( rawMag.x, rawMag.y, rawMag.z );
    // Serial.println(rawMag->x);
    Mag = (rawMag - MagOffset) * MagGain;
    Mag = changeAxisSign( Mag, -1, 1, -1 );
    CurrentMag = CurrentMag*MAG_INTEG + Mag*(1-MAG_INTEG);
    // Serial.println(CurrentMag.x);

    return MyVector_to_FusionVector( CurrentMag );
}

inline FusionVector MyVector_to_FusionVector(const vector& vec) {
    /* Convert from MyVector::vector to FusionVector-object. */
    FusionVector res;

    res.axis.x = vec.x;
    res.axis.y = vec.y;
    res.axis.z = vec.z;

    return res;
}

inline vector changeAxisSign(const vector& vec, int xSign, int ySign, int zSign) {
    return vector( vec.x * xSign, vec.y * ySign, vec.z * zSign );
}

float updateTimeStamp(void) {
    /* Calculate timestep and convert from microseconds to seconds. */
    static const float MICROSECONDS_TO_SECONDS = 1.0f / 1000000.0f;
    static uint32_t IMU_timeStamp = micros();
    static uint32_t IMU_previousTimeStamp;

    IMU_timeStamp = micros();
    float dt = static_cast<float>((IMU_timeStamp-IMU_previousTimeStamp)*MICROSECONDS_TO_SECONDS); 
    IMU_previousTimeStamp = IMU_timeStamp;

    return dt;
}

void updateJoystickAxes(const FusionAhrs *const ahrs, Joystick *const joy, USBCommsJoystick *const usb) {
    /*Update the joystick-axes using the AHRS angle data. */
    FusionEuler euler = FusionQuaternionToEuler( FusionAhrsGetQuaternion( ahrs ));

    joy->setAxis( remap_yaw(euler.angle.yaw, AxisOffset.x), X);
    joy->setAxis( euler.angle.pitch + AxisOffset.y, Y);
    joy->setAxis( euler.angle.roll + AxisOffset.z, Z);

    usb->updateHIDreport(joy);
}

void AHRS_check(void) {
  static FusionVector gyroscope;
  static FusionVector accelerometer;
  static FusionVector magnetometer; 
  static float deltaTime;
  /*
    TODO: Try to reduce code duplication.
  */

  /* If acceleration, gyroscopy and magnetometer data are ready. 
   Accelerometer and gyroscope run with higher sample rate, magnetometer with 20 Hz. */
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()) {
    deltaTime = updateTimeStamp(); 

    accelerometer = readAcceleration(rawAcc);   
    magnetometer = readMagneticField(rawMag);
    gyroscope = readGyroscope(rawGyro);

    // Compensate for the long-term gyroscope drift.
    gyroscope = FusionOffsetUpdate(&AHRS_gyro_offset, gyroscope); 

    // Run the AHRS-algorithm.
    FusionAhrsUpdate(&AHRS, gyroscope, accelerometer, magnetometer, deltaTime); 
    CompassHeading = FusionCompassCalculateHeading(accelerometer, magnetometer);
  }

  /* If acceleration and gyroscope data are ready. */
  else if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    deltaTime = updateTimeStamp(); 

    accelerometer = readAcceleration(rawAcc);   
    gyroscope = readGyroscope(rawGyro);
    
    // Compensate for the long-term gyroscope drift.
    gyroscope = FusionOffsetUpdate(&AHRS_gyro_offset, gyroscope); 

    // Run the AHRS-algorithm
    FusionAhrsUpdateNoMagnetometer(&AHRS, gyroscope, accelerometer, deltaTime); 
  }
}
/*
------------ END OF AHRS PART ---------------------------------------
*/




void setup() {
    while (!Serial) {}
    Serial.begin(SERIAL_BAUDRATE);

    IMU.setGyroscopeSettings( LSM9DS1_ODR_G_238HZ, LSM9DS1_FS_G_500DPS );
    IMU.setAccelerometerSettings( LSM9DS1_ODR_XL_119HZ, LSM9DS1_FS_XL_4G );
    IMU.begin(); // Start the STM LSM9DS1 inertial unit.

    // Madgwick fusion library initialization.
    FusionOffsetInitialise(&AHRS_gyro_offset, SAMPLE_RATE);
    FusionAhrsInitialise(&AHRS);
    FusionAhrsSetSettings(&AHRS, &AHRSsettings);

    usb_comms.setSettings(usb_comms.NO_AUTOSEND, usb_comms.SEND_NONBLOCKING);
    joystick.setAxisRange( -180, 180, X );  // left-right, yaw-axis. Range [-180, 180] degrees. 
    joystick.setAxisRange( -90, 90, Y );    // up-down, pitch-axis. Range [-90, 90] degrees.
    joystick.setAxisRange( -90, 90, Z );  // roll left-right, roll-axis. Range [-90, 90] degrees.


    if (!kv_store_initialized()) {
      while (true) {
        Serial.println("Unrecoverable error with KVStore...");
        delay(2000);
      }
    }

    kv_store_load_calibration(kv_keys[cal_mag_offset], MagOffset, MagOffset_default); 
    kv_store_load_calibration(kv_keys[cal_mag_gain], MagGain, MagGain_default); 
    kv_store_load_calibration(kv_keys[cal_acc_offset], AccOffset, AccOffset_default); 
    kv_store_load_calibration(kv_keys[cal_acc_gain], AccGain, AccGain_default); 
    kv_store_load_calibration(kv_keys[cal_gyro_offset], GyroOffset, GyroOffset_default); 
    kv_store_load_calibration(kv_keys[cal_gyro_gain], GyroGain, GyroGain_default); 
    kv_store_load_calibration(kv_keys[cal_euler_output_offset], AxisOffset, AxisOffset_default);

    Serial.println("System reset.");
}

void loop() {
  static uint32_t serial_output_timer = millis();
  static uint32_t serial_input_timer = millis();
  
  //Serial.println("AHRS");
  AHRS_check();
  //Serial.println("Update joystick.");
  updateJoystickAxes(&AHRS, &joystick, &usb_comms);
  //Serial.println("Send HID-report.");
  usb_comms.update();

  if (millis() - serial_output_timer > 200) {
    serial_output_timer = millis();
    print_output();

    //const auto [sendBlocking, autoSend] = usb_comms.getSettings();
    //Serial.println(autoSend);
    //Serial.println(sendBlocking);

    //const auto [sendBlocking_ptr, autoSend_ptr] = usb_comms.getSettingsPtr();
    //Serial.println(reinterpret_cast<int>(autoSend_ptr));
    //Serial.println(reinterpret_cast<int>(sendBlocking_ptr));
  }

  if (millis() - serial_input_timer > 2000) {
    serial_input_timer = millis();
    check_serial_input();
  }
}
