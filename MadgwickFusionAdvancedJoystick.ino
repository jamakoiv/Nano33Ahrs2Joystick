#include <map>
#include <string>
#include <vector>
#include <cstring>
#include <kvstore_global_api.h>

#include <Fusion.h>
#include <USBJoystick.h>

#include "MyVector.h"
#include "LSM9DS1.h"
#include "serial_commands.h"

/*
  TODO: Using MyVector::vector and std::vector together is confusing and asking for trouble.
  TODO: Maybe try having less global variables...
*/

USBJoystick joystick;

auto SerialOutputMode = SERIAL_PRINT_AHRS;

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

void updateJoystickAxes(const FusionAhrs *const ahrs) {
    /*Update the joystick-axes using the AHRS angle data. */
    FusionEuler euler = FusionQuaternionToEuler( FusionAhrsGetQuaternion( ahrs ));

    joystick.setXAxis( euler.angle.yaw );
    joystick.setYAxis( euler.angle.pitch );
    joystick.setZAxis( euler.angle.roll );

    joystick.update();
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


/*
 ---------------- SERIAL OUTPUT PART ------------------------------
*/
void printAHRSeuler(void);
void printNothing(void);

void printAccCalib(void);
void printMagCalib(void);
void printGyroCalib(void);

void printAccRaw(void);
void printMagRaw(void);
void printGyroRaw(void);

using output_func_ptr_t = void (*)(void);
std::map<uint8_t, output_func_ptr_t> output_functions = {
    {SERIAL_PRINT_AHRS, &printAHRSeuler},
    {SERIAL_PRINT_NOTHING, &printNothing},

    {SERIAL_PRINT_ACC_CALIB, &printAccCalib},
    {SERIAL_PRINT_MAG_CALIB, &printMagCalib},
    {SERIAL_PRINT_GYRO_CALIB, &printGyroCalib},

    {SERIAL_PRINT_ACC_RAW, &printAccRaw},
    {SERIAL_PRINT_MAG_RAW, &printMagRaw},
    {SERIAL_PRINT_GYRO_RAW, &printGyroRaw},
    };

void printAHRSeuler(void) {
    const FusionEuler euler = FusionQuaternionToEuler( FusionAhrsGetQuaternion( &AHRS ) );

    std::string str = "Roll: " + std::to_string(euler.angle.roll) + ", " +
                      "Pitch: " + std::to_string(euler.angle.pitch) + ", " +
                      "Yaw: " + std::to_string(euler.angle.yaw) + ", " +
                      "Compass: " + std::to_string(CompassHeading);
    Serial.println(str.c_str());
}


void printNothing(void) {
    return;
}

void printAccCalib(void) {
    Serial.println(CurrentAcc.to_string().c_str());
}
void printMagCalib(void) {
    Serial.println(CurrentMag.to_string().c_str());
}
void printGyroCalib(void) {
    Serial.println(CurrentGyro.to_string().c_str());
}

void printAccRaw(void) {
    Serial.println(rawAcc.to_string().c_str());
}
void printMagRaw(void) {
    Serial.println(rawMag.to_string().c_str());
}
void printGyroRaw(void) {
    Serial.println(rawGyro.to_string().c_str());
}

void print_output(void) {
  auto it = output_functions.find(SerialOutputMode);
  if (it == output_functions.end()) {
    Serial.println("Output command not found.");
  } else {
    (it->second)();
  }
  //output_functions[SerialOutputMode]();
}
/*
-------------------- END OF SERIAL OUTPUT PART ------------------
*/

// Updates deltaTime variable.
/*
-------------------- SERIAL INPUT PART --------------------  
*/

// Forward declarations for the map.
void mag_set_calib(std::string input);
void mag_get_calib(std::string input);

void acc_set_calib(std::string input);
void acc_get_calib(std::string input);

void gyro_set_calib(std::string input);
void gyro_get_calib(std::string input);

void set_print_nothing(std::string input);
void set_print_ahrs(std::string input);
void set_print_mag_raw(std::string input);
void set_print_mag_calib(std::string input);
void set_print_acc_raw(std::string input);
void set_print_acc_calib(std::string input);
void set_print_gyro_raw(std::string input);
void set_print_gyro_calib(std::string input);

void kv_store_reset(std::string input);

using input_func_ptr_t = void (*)(std::string input);
std::map<uint8_t, input_func_ptr_t> input_functions = { 
          {SERIAL_MAG_SET_CALIB, &mag_set_calib},
          {SERIAL_MAG_GET_CALIB, &mag_get_calib},

          {SERIAL_ACC_SET_CALIB, &acc_set_calib},
          {SERIAL_ACC_GET_CALIB, &acc_get_calib},

          {SERIAL_GYRO_SET_CALIB, &gyro_set_calib},
          {SERIAL_GYRO_GET_CALIB, &gyro_get_calib},

          {SERIAL_PRINT_NOTHING, &set_print_nothing},
          {SERIAL_PRINT_AHRS, &set_print_ahrs},
          {SERIAL_PRINT_MAG_RAW, &set_print_mag_raw},
          {SERIAL_PRINT_MAG_CALIB, &set_print_mag_calib},
          {SERIAL_PRINT_ACC_RAW, &set_print_acc_raw},
          {SERIAL_PRINT_ACC_CALIB, &set_print_acc_calib},
          {SERIAL_PRINT_GYRO_RAW, &set_print_gyro_raw},
          {SERIAL_PRINT_GYRO_CALIB, &set_print_gyro_calib},

          {SERIAL_RESET_KVSTORE, &kv_store_reset} };

std::vector<std::string> split_input(std::string input, const std::string& delimiter) {
/*
  Split string at 'delimiter' and return the pieces in a std::vector<std::string>.
*/
  std::vector<std::string> res;
    
  auto pos = input.find(delimiter);

  while (pos != std::string::npos) {
    res.emplace_back(input.substr(0, pos));
    input.erase(0, pos+1);
    pos = input.find(delimiter);
  }
  
  // Handle the last element if there is no delimiter at the end of the input string.
  if (input.empty()) { 
    return res;
  } else {
    res.emplace_back(input);
    return res;
  }
}

std::vector<float> split_and_strtof(std::string input, const std::string& delimiter) {
/*
  Split string, convert elements to float and return them as std::vector<float>.
*/
  std::vector<float> res;
  auto input_str_vec = split_input(input, delimiter);
  for (std::string str : input_str_vec) {
    res.emplace_back(strtof(str.c_str(), NULL));
  }

  return res;
}

void set_calib_helper(const std::vector<float>& data, vector& offset, vector& gain) {
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

bool serial_handshake(void) {
/*
  Call and response.   
*/
  static char serialBuffer[SERIAL_READ_BUFFER_SIZE];

  strncpy(serialBuffer, NULL, SERIAL_READ_BUFFER_SIZE);
  if (!Serial.available()) {
    return false;
  }

  Serial.readBytes(serialBuffer, SERIAL_HANDSHAKE.length());
  if (std::string(serialBuffer) == SERIAL_HANDSHAKE) {
    Serial.println(SERIAL_HANDSHAKE.c_str());
    return true;
  } else {
    return false;
  }
}

std::string read_serial_input(void) {
/*

*/
  static char serialBuffer[SERIAL_READ_BUFFER_SIZE];

  strncpy(serialBuffer, NULL, SERIAL_READ_BUFFER_SIZE);
  int bytes_read = Serial.readBytesUntil('\n', serialBuffer, SERIAL_READ_BUFFER_SIZE);

  if (bytes_read == 0) {
    return std::string("");
  } else {
    return std::string(serialBuffer);
  }
}

void execute_command(std::vector<std::string> params) {
  // Guard against inputs which would crash the program at the "(it->second)(params[1])"
  if (params.size() == 1) params.emplace_back(" "); 

  uint32_t command= static_cast<uint8_t>(strtol(params[0].c_str(), NULL, 16));

  auto it = input_functions.find(command);
  if (it == input_functions.end()) {
    Serial.print("Command not found: 0x");
    Serial.println(command, HEX);
  } else {
    (it->second)(params[1]);
  }
}

void check_serial_input(void) {
/* 
  Check for commands in serial. 
*/
  static const std::string OPTIONS_DELIMITER = ";";

  if (!serial_handshake()) {
    return;
  }

  delay(2000);// Small delay in case the received message is 
              // incomplete when we check Serial.available.
              // Make larger if you want to send data manually via terminal.

  auto input = read_serial_input();
  auto input_params = split_input(input, OPTIONS_DELIMITER);
  execute_command(input_params);
}

void mag_set_calib(std::string input) {
  set_calib_helper(split_and_strtof(input, ","), MagOffset, MagGain);
  kv_store_save_calibration("MagOffset", MagOffset);
  kv_store_save_calibration("MagGain", MagGain);
  Serial.println("Calibration set.");
}

void mag_get_calib(std::string input) {
  SerialOutputMode = SERIAL_PRINT_NOTHING;
  std::string str = "Magnetic offset (x,y,z): " + MagOffset.to_string()
                  + "; Magnetic gain (x,y,z): " + MagGain.to_string();
  Serial.println(str.c_str());
}

void acc_set_calib(std::string input) {
  set_calib_helper(split_and_strtof(input, ","), AccOffset, AccGain);
  kv_store_save_calibration("AccOffset", AccOffset);
  kv_store_save_calibration("AccGain", AccGain);
  Serial.println("Calibration set.");
}

void acc_get_calib(std::string input) {
  SerialOutputMode = SERIAL_PRINT_NOTHING;
  std::string str = "Accelerometer offset (x,y,z): " + AccOffset.to_string();
                  + "; Accelerometer gain (x,y,z): " + AccGain.to_string();
  Serial.println(str.c_str());
}

void gyro_set_calib(std::string input) {
  set_calib_helper(split_and_strtof(input, ","), GyroOffset, GyroGain);
  kv_store_save_calibration("GyroOffset", GyroOffset);
  kv_store_save_calibration("GyroGain", GyroGain);
  Serial.println("Calibration set.");
}

void gyro_get_calib(std::string input) {
  SerialOutputMode = SERIAL_PRINT_NOTHING;
  std::string str = "Gyroscope offset (x,y,z): " + GyroOffset.to_string();
                  + "; Gyroscope gain (x,y,z): " + GyroGain.to_string();
  Serial.println(str.c_str());
}

/* functions for settings print output mode. */
void set_print_nothing(std::string input) {
  SerialOutputMode = SERIAL_PRINT_NOTHING;
  Serial.println("Output-mode set to NOTHING.");
}
void set_print_ahrs(std::string input) {
  SerialOutputMode = SERIAL_PRINT_AHRS;
  Serial.println("Output-mode set to AHRS.");
}
void set_print_mag_raw(std::string input) {
  SerialOutputMode = SERIAL_PRINT_MAG_RAW;
  Serial.println("Output-mode set to MAGNETOMETER-RAW.");
}
void set_print_mag_calib(std::string input) {
  SerialOutputMode = SERIAL_PRINT_MAG_CALIB;
  Serial.println("Output-mode set to MAGNETOMETER-CALIBRATED.");
}
void set_print_acc_raw(std::string input) {
  SerialOutputMode = SERIAL_PRINT_ACC_RAW;
  Serial.println("Output-mode set to ACCELEROMETER-RAW.");
}
void set_print_acc_calib(std::string input) {
  SerialOutputMode = SERIAL_PRINT_ACC_CALIB;
  Serial.println("Output-mode set to ACCELEROMETER-CALIBRATED.");
}
void set_print_gyro_raw(std::string input) {
  SerialOutputMode = SERIAL_PRINT_GYRO_RAW;
  Serial.println("Output-mode set to GYROSCOPE-RAW.");
}
void set_print_gyro_calib(std::string input) {
  SerialOutputMode = SERIAL_PRINT_GYRO_CALIB;
  Serial.println("Output-mode set to GYROSCOPE-RAW.");
}
/*
-------------------- END OF SERIAL INPUT PART --------------------
*/


/*
-------------------- KV-STORE PART --------------------
*/

const uint8_t KV_BUFFER_SIZE = 64;
const std::string kv_path = "/kv/";
const std::string kv_init_key = kv_path + "KVStore_global_api_init";

bool kv_store_initialized(void) {
    kv_info_t info; 

    auto res = kv_get_info(kv_init_key.c_str(), &info);
    
    if (res == MBED_ERROR_ITEM_NOT_FOUND) {
        Serial.println("KVStore not initialized. Resetting KVStore...");
        res = kv_reset(kv_path.c_str());
        if (res != MBED_SUCCESS) {
            return false;
        }
        
        res = kv_set(kv_init_key.c_str(), "1", 1, 0);
        if (res != MBED_SUCCESS) {
            Serial.println("Error setting init key to KVStore.");
            return false;
        }
        Serial.println("KVStore reset successfully.");
        return true;
    } else {
        Serial.println("KVStore seems ok...");
        return true;
    }
}

bool kv_store_save_calibration(const std::string& key, const vector& data) {
    auto data_str = data.to_string();
    auto full_key = kv_path + key;

    auto res = kv_set(full_key.c_str(), data_str.c_str(), data_str.length(), 0);

    if (res == MBED_SUCCESS) return true;
    else return false;
}

bool kv_store_load_calibration(const std::string& key, vector& calib, vector& factory_default) {
/*
  Load calibration values from the KVstore. If the KVStore key does not exist, 
  set calibration to given factory defaults.

  Returns true if calibration was loaded from KVStore succesfully.
  Returns false if loading failed and defaults were used instead.
*/
    static char kv_get_buffer[KV_BUFFER_SIZE];
    static std::string VALUES_DELIMITER = ",";

    size_t bytes_received;
    kv_info_t info;
    auto full_key = kv_path + key;

    calib = factory_default; 

    auto res = kv_get_info(full_key.c_str(), &info);
    if (res == MBED_ERROR_ITEM_NOT_FOUND) return false;

    res = kv_get(full_key.c_str(), kv_get_buffer, info.size, &bytes_received);
    if (res != MBED_SUCCESS) return false;

    auto values = split_and_strtof(std::string(kv_get_buffer), VALUES_DELIMITER);
    if (values.size() != 3) return false;
    
    calib = vector(values[0], values[1], values[2]);
    return true;
}

// kv_store_reset function has to match the 'std::map<...> input_functions' signature.
void kv_store_reset(std::string input) {
    
    // TODO: kv_store_initialized calls kv_reset as well. 
    kv_reset(kv_path.c_str()); 

    if (!kv_store_initialized()) {
      while (true) {
        Serial.println("Unrecoverable error resetting KVStore.");
        delay(2000);
      }
    }

    // TODO: KVStore keys are hardcoded and used in three different places.
    //       Simple typo will give hard to track bugs.
    kv_store_save_calibration("MagOffset", MagOffset_default);
    kv_store_save_calibration("MagGain", MagGain_default);
    kv_store_save_calibration("AccOffset", AccOffset_default);
    kv_store_save_calibration("AccGain", AccGain_default);
    kv_store_save_calibration("GyroOffset", GyroOffset_default);
    kv_store_save_calibration("GyroGain", GyroGain_default);

    MagOffset = MagOffset_default;
    MagGain = MagGain_default;
    AccOffset = AccOffset_default;
    AccGain = AccGain_default;
    GyroOffset = GyroOffset_default;
    GyroGain = GyroGain_default;
}
/*
-------------------- END OF KV-STORE PART --------------------
*/


void setup() {
    Serial.begin(SERIAL_BAUDRATE);

    IMU.setGyroscopeSettings( LSM9DS1_ODR_G_238HZ, LSM9DS1_FS_G_500DPS );
    IMU.setAccelerometerSettings( LSM9DS1_ODR_XL_119HZ, LSM9DS1_FS_XL_4G );
    IMU.begin(); // Start the STM LSM9DS1 inertial unit.

    // Madgwick fusion library initialization.
    FusionOffsetInitialise(&AHRS_gyro_offset, SAMPLE_RATE);
    FusionAhrsInitialise(&AHRS);
    FusionAhrsSetSettings(&AHRS, &AHRSsettings);

    joystick.autoSend = false;
    joystick.sendBlocking = false;
    joystick.setXAxisRange( -180, 180 );  // left-right, yaw-axis. Range [-180, 180] degrees. 
    joystick.setYAxisRange( -90, 90 );    // up-down, pitch-axis. Range [-90, 90] degrees.
    joystick.setZAxisRange( -90, 90 );  // roll left-right, roll-axis. Range [-90, 90] degrees.

    //joystick.setXAxisRange( -90, 90 );  // left-right, yaw-axis. Range [-90, 90] degrees. 
    //joystick.setYAxisRange( -90, 90 );    // up-down, pitch-axis. Range [-90, 90] degrees.
    //joystick.setZAxisRange( -180, 180 );  // roll left-right, roll-axis. Range [-180, 180] degrees.

    if (!kv_store_initialized()) {
      while (true) {
        Serial.println("Unrecoverable error with KVStore...");
        delay(2000);
      }
    }

    kv_store_load_calibration("MagOffset", MagOffset, MagOffset_default); 
    kv_store_load_calibration("MagGain", MagGain, MagGain_default); 
    kv_store_load_calibration("AccOffset", AccOffset, AccOffset_default); 
    kv_store_load_calibration("AccGain", AccGain, AccGain_default); 
    kv_store_load_calibration("GyroOffset", GyroOffset, GyroOffset_default); 
    kv_store_load_calibration("GyroGain", GyroGain, GyroGain_default); 

    Serial.println("System reset.");
}

void loop() {
  static uint32_t serial_output_timer = millis();
  static uint32_t serial_input_timer = millis();
  
  AHRS_check();
  updateJoystickAxes(&AHRS);

  if (millis() - serial_output_timer > 100) {
    serial_output_timer = millis();
    print_output();
  }

  if (millis() - serial_input_timer > 200) {
    serial_input_timer = millis();
    check_serial_input();
  }
}
