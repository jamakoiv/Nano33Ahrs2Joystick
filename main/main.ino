#include "src/Fusion/Fusion.h"
#include "src/USBJoystick/USBCommsJoystick.h"
#include "src/LSM9DS1/LSM9DS1.h"

#include "src/serial_comms.h"
#include "src/kv_storage.h"
#include "src/utils.h"

/*
 * TODO: Maybe try having less global variables...
 * TODO: Axis ranges are defined in multiple places with magic numbers. Replace with
 *      central definition.
*/

USBCommsJoystick usb_comms;
Joystick joystick;
int SerialOutputMode = SERIAL_PRINT_MAG_CALIB;

char serial_tmp_buffer[256];

/*
  Fusion-library objects and variables.
*/
const uint32_t SAMPLE_RATE = 238;
FusionOffset AHRS_gyro_offset;
FusionAhrs AHRS;
float CompassHeading;

// Set AHRS algorithm settings
const FusionAhrsSettings AHRSsettings = {
        .convention = FusionConventionNed, /* North-East-Down */
        .gain = 0.50f,
        // .gain = 3.00f,
        .gyroscopeRange = 500.0,  // In degrees per second.
        .accelerationRejection = 10.0f,
        .magneticRejection = 10.0f,
        .recoveryTriggerPeriod = 5*SAMPLE_RATE
} ;

// Variables for acceleration values and calibrations.
FusionVector acc_raw;
FusionVector acc_calibrated;
FusionVector acc_gain;
FusionVector acc_gain_default {1.0, 1.0, 1.0};
FusionVector acc_offset;
FusionVector acc_offset_default = {0.0325f, 0.0306f, 0.01819f}; 

// Variables for gyroscope values and calibrations.
FusionVector gyro_raw;
FusionVector gyro_calibrated;
FusionVector gyro_gain;
FusionVector gyro_gain_default {1.125f, 1.125f, 1.125f};
FusionVector gyro_offset;
FusionVector gyro_offset_default = {0.50019f, 0.68556f, -0.13808f}; // 238 Hz values

// Variables for magnetometer values and calibrations.
FusionVector mag_raw;
FusionVector mag_calibrated;
FusionMatrix soft_iron;
FusionMatrix soft_iron_default = {
  1.0/46.34, 0.0, 0.0,
  0.0, 1.0/45.84, 0.0,
  0.0, 0.0, 1.0/44.18
};
FusionVector hard_iron;
FusionVector hard_iron_default = {2.97, -26.39, 14.13};

// x0=3.2874, y0=-25.4467, z0=14.1876, a=46.3417, b=45.8475, c=44.1814:software

/* AxisOffset(yaw, pitch, roll). Add constant offset to the AHRS-Euler 
 * output as degrees. Generally you should leave these zero and mitigate 
 * output default position problems in the OS software side.
 * INFO: Due to FusionVector these must still be accessed as vec.axis.x etc.
 */
FusionVector AxisOffset;
FusionVector AxisOffset_default = {0.0f, 0.0f, 0.0f};


void readAcceleration() {
    IMU.readAcceleration(acc_raw.axis.x, acc_raw.axis.y, acc_raw.axis.z);
    // acc_calibrated = FusionVectorHadamardProduct(FusionVectorSubtract(acc_raw, acc_offset), acc_gain);
    acc_calibrated = FusionVectorHadamardProduct(FusionVectorSubtract(acc_raw, acc_offset_default), acc_gain_default);
    acc_calibrated = changeAxisSign(acc_calibrated, 1, 1, -1);
}

void readGyroscope() {
    IMU.readGyroscope(gyro_raw.axis.x, gyro_raw.axis.y, gyro_raw.axis.z);
    // gyro_calibrated = FusionVectorHadamardProduct(FusionVectorSubtract(gyro_raw, gyro_offset), gyro_gain);
    gyro_calibrated = FusionVectorHadamardProduct(FusionVectorSubtract(gyro_raw, gyro_offset_default), gyro_gain_default);
    gyro_calibrated = changeAxisSign(gyro_calibrated, 1, 1, -1 );
}

void readMagneticField() {
    IMU.readMagneticField(mag_raw.axis.x, mag_raw.axis.y, mag_raw.axis.z);
    // mag_calibrated = FusionCalibrationMagnetic(mag_raw, soft_iron, hard_iron);
    mag_calibrated = FusionCalibrationMagnetic(mag_raw, soft_iron_default, hard_iron_default);
    mag_calibrated = changeAxisSign(mag_calibrated, -1, 1, -1);
}


inline FusionVector changeAxisSign(FusionVector vec, int xSign, int ySign, int zSign) {
    FusionVector res = {vec.axis.x * xSign, vec.axis.y * ySign, vec.axis.z * zSign};
    return res;
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


void AHRS_check(void) {
  /*
   * Retrieve data from sensors and feed them to the AHRS algorithm 
  */
  //  TODO: Try to reduce code duplication.

  static float deltaTime;

  /* 
  Accelerometer and gyroscope run with higher sample rate than the magnetometer,
  so we run the AHRS algorithm without the magnetometer if it is not ready.
  */
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()) {
    deltaTime = updateTimeStamp(); 

    readAcceleration();   
    readMagneticField();
    readGyroscope();

    // Compensate for the long-term gyroscope drift.
    gyro_calibrated = FusionOffsetUpdate(&AHRS_gyro_offset, gyro_calibrated); 

    // Run the AHRS-algorithm.
    FusionAhrsUpdate(&AHRS, gyro_calibrated, acc_calibrated, mag_calibrated, deltaTime); 
    CompassHeading = FusionCompassCalculateHeading(FusionConventionNed, acc_calibrated, mag_calibrated);
  }

  else if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    deltaTime = updateTimeStamp(); 

    readAcceleration();
    readGyroscope();
    
    gyro_calibrated = FusionOffsetUpdate(&AHRS_gyro_offset, gyro_calibrated); 
    FusionAhrsUpdateNoMagnetometer(&AHRS, gyro_calibrated, acc_calibrated, deltaTime); 
  }
}
/*
------------ END OF AHRS PART ---------------------------------------
*/


void updateJoystickAxes(const FusionAhrs *const ahrs, Joystick *const joy, USBCommsJoystick *const usb) {
    /*
     * Update the joystick-axes using the AHRS angle data. 
    */

    FusionEuler euler = FusionQuaternionToEuler( FusionAhrsGetQuaternion( ahrs ));

    joy->setAxis( remap_yaw(euler.angle.yaw, AxisOffset.axis.x), X);
    joy->setAxis( euler.angle.pitch + AxisOffset.axis.y, Y);
    joy->setAxis( euler.angle.roll + AxisOffset.axis.z, Z);

    usb->updateHIDreport(joy);
}


void setup() {
    while (!Serial) {}
    Serial.begin(SERIAL_BAUDRATE);
    delay(100);
    Serial.println("Starting board.");

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

    Serial.println("Retrieve calibrations from KVStore.");
    delay(100);

    // kv_storage_reset({}); // Reset the KVStore to factory defaults.
    kv_store_load_calibration(kv_keys[cal_mag_offset], hard_iron, hard_iron_default);
    kv_store_load_calibration(kv_keys[cal_mag_gain], soft_iron, soft_iron_default);
    kv_store_load_calibration(kv_keys[cal_acc_offset], acc_offset, acc_offset_default);
    kv_store_load_calibration(kv_keys[cal_acc_gain], acc_gain, acc_gain_default);
    kv_store_load_calibration(kv_keys[cal_gyro_offset], gyro_offset, gyro_offset_default);
    kv_store_load_calibration(kv_keys[cal_gyro_gain], gyro_gain, gyro_gain_default);
    kv_store_load_calibration(kv_keys[cal_euler_output_offset], AxisOffset, AxisOffset_default);

    Serial.println("System reset.");
    delay(100);
}

vector<command_t> check_serial_input(void) {
    // NOTE: We assume 
    const char EOT = 0x04;

    std::strncpy(serial_tmp_buffer, NULL, 256);
    vector<command_t> commands;

    while (int bytes_read = Serial.readBytesUntil(EOT, serial_tmp_buffer, 256)) {
        std::string msg(serial_tmp_buffer);
        // TODO: Currently our processing assumes the EOT-byte is present in the message,
        // but readBytesUntil does not include the terminator-byte in the read data.
        msg.push_back(EOT); 
        Serial.println(msg.c_str());

        command_t cmd = retrieve_command(msg);
        Serial.println(cmd.id);
        Serial.println(cmd.n_params);
        commands.push_back(cmd);

        std::strncpy(serialBuffer, NULL, 256);
    }

    return commands;
}

void loop() {
  static uint32_t serial_output_timer = millis();
  static uint32_t serial_input_timer = millis();
  
  /*
   * Main functionality. Run the AHRS algorithm and update the current orientation to the joystick.
   */
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

  // NOTE: For Arduino Nano33 BLE the serial input buffer is 256 bytes.
  // Currently all messages fit nicely in this limit, and we can assume every message fits
  // in the buffer and keep the delay. 
  // If things change we must get rid of the delay and read input continuously.
  if (Serial.available()) {
    Serial.println("Serial available");
    delay(100);

    std::vector<command_t> commands = check_serial_input();
    execute_commands(commands);
  }
}

