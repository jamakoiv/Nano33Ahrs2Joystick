#include <Fusion.h>
#include <USBJoystick.h>
#include <map>
#include <string>
#include <vector>
#include <cstring>

#include "MyVector.h"
#include "LSM9DS1.h"

/*
  TODO: Using MyVector::vector and std::vector together is confusing and asking for trouble.
  TODO: Maybe try having less global variables...
*/

USBJoystick joystick;

enum {
    SERIAL_PRINT_NOTHING = 0x10,
    SERIAL_PRINT_MAG_RAW = 0x11,
    SERIAL_PRINT_MAG_CALIB = 0x12,
    SERIAL_PRINT_ACC_RAW = 0x13,
    SERIAL_PRINT_ACC_CALIB = 0x14,
    SERIAL_PRINT_GYRO_RAW = 0x15,
    SERIAL_PRINT_GYRO_CALIB = 0x16,
    SERIAL_PRINT_AHRS = 0x20,

    SERIAL_MAG_SET_OFFSET = 0x30,
    SERIAL_MAG_GET_OFFSET = 0x31,
    SERIAL_MAG_SET_GAIN = 0x32,
    SERIAL_MAG_GET_GAIN = 0x33,
    SERIAL_ACC_SET_OFFSET = 0x40,
    SERIAL_ACC_GET_OFFSET = 0x41,
    SERIAL_ACC_SET_GAIN = 0x42,
    SERIAL_ACC_GET_GAIN = 0x43,
    SERIAL_GYRO_SET_OFFSET = 0x50,
    SERIAL_GYRO_GET_OFFSET = 0x51,
    SERIAL_GYRO_SET_GAIN = 0x52,
    SERIAL_GYRO_GET_GAIN = 0x53,

    SERIAL_BAUDRATE = 57600,
    SERIAL_READ_BUFFER_SIZE = 65,
} ;
auto SerialOutputMode = SERIAL_PRINT_AHRS;

/*
 IMU measurement variables and calibration.
*/
using MyVector::vector;

/*
  Fusion-library objects and variables.
*/
const uint32_t SAMPLE_RATE = 238;
FusionOffset offset;
FusionAhrs AHRS;

FusionVector gyroscope;
FusionVector accelerometer;
FusionVector magnetometer; 

uint32_t IMU_timeStamp, IMU_previousTimeStamp;
float deltaTime;

// Set AHRS algorithm settings
const FusionAhrsSettings AHRSsettings = {
        .gain = 0.50f,
        .accelerationRejection = 10.0f,
        .magneticRejection = 20.0f,
        .rejectionTimeout = 2*SAMPLE_RATE
        //.rejectionTimeout = 0
} ;

// Variables for acceleration values and calibrations.
vector Acc; 
vector rawAcc;
vector CurrentAcc;
vector AccGain(1.00f, 1.00f, 1.00f);
vector AccOffset(0.0325f, 0.0306f, 0.01819f); 

// Variables for gyroscope values and calibrations.
vector Gyro;
vector rawGyro;
vector CurrentGyro;
vector GyroGain(1.125f, 1.125f, 1.125f); 
vector GyroOffset(-0.50019f, -0.68556f, 0.13808f);     // 238 Hz values

// Variables for magnetometer values and calibrations.
vector Mag;
vector rawMag;
vector CurrentMag;
vector MagGain(1.0f, 1.0f, 1.0f);
vector MagOffset(-7.257f, 39.747f, -11.817f);

const float GYRO_INTEG = 0.90f;
const float ACC_INTEG = 0.90f;
const float MAG_INTEG = 0.90f;

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


void updateTimeStamp(void) {
    /* Calculate timestep and convert from microseconds to seconds. */
    static const float MICROSECONDS_TO_SECONDS = 1.0f / 1000000.0f;

    // Variables defined globally, bad...
    IMU_timeStamp = micros();
    deltaTime = static_cast<float>((IMU_timeStamp-IMU_previousTimeStamp)*MICROSECONDS_TO_SECONDS); 
    IMU_previousTimeStamp = IMU_timeStamp;
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
  /* If acceleration, gyroscopy and magnetometer data are ready. 
   Accelerometer and gyroscope run with higher sample rate, magnetometer with 20 Hz. */
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()) {
    updateTimeStamp(); // Updates deltaTime variable.

    accelerometer = readAcceleration(rawAcc);   
    magnetometer = readMagneticField(rawMag);
    gyroscope = readGyroscope(rawGyro);

    // Compensate for the long-term gyroscope drift.
    gyroscope = FusionOffsetUpdate( &offset, gyroscope ); 

    // Run the AHRS-algorithm.
    FusionAhrsUpdate( &AHRS, gyroscope, accelerometer, magnetometer, deltaTime ); 

    updateJoystickAxes( &AHRS );
  }

  /* If acceleration and gyroscope data are ready. */
  else if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    updateTimeStamp(); // Updates deltaTime variable.

    accelerometer = readAcceleration(rawAcc);   
    gyroscope = readGyroscope(rawGyro);
    
    // Compensate for the long-term gyroscope drift.
    gyroscope = FusionOffsetUpdate( &offset, gyroscope ); 

    // Run the AHRS-algorithm
    FusionAhrsUpdateNoMagnetometer( &AHRS, gyroscope, accelerometer, deltaTime ); 

    updateJoystickAxes( &AHRS );
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

std::map<uint8_t, void (*)(void)> output_functions = {
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
                      "Yaw: " + std::to_string(euler.angle.yaw) + ", ";
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


/*
-------------------- SERIAL INPUT PART --------------------  
*/

// Forward declarations for the map.
void mag_set_offset(std::string input);
void mag_set_gain(std::string input);
void mag_get_offset(std::string input);
void mag_get_gain(std::string input);

void acc_set_offset(std::string input);
void acc_set_gain(std::string input);
void acc_get_offset(std::string input);
void acc_get_gain(std::string input);

void gyro_set_offset(std::string input);
void gyro_set_gain(std::string input);
void gyro_get_offset(std::string input);
void gyro_get_gain(std::string input);

void set_print_nothing(std::string input);
void set_print_ahrs(std::string input);
void set_print_mag_raw(std::string input);
void set_print_mag_calib(std::string input);
void set_print_acc_raw(std::string input);
void set_print_acc_calib(std::string input);
void set_print_gyro_raw(std::string input);
void set_print_gyro_calib(std::string input);

std::map<uint8_t, void (*)(std::string input)> input_functions = { 
          {SERIAL_MAG_SET_OFFSET, &mag_set_offset},
          {SERIAL_MAG_SET_GAIN, &mag_set_gain},
          {SERIAL_MAG_GET_OFFSET, &mag_get_offset},
          {SERIAL_MAG_GET_GAIN, &mag_get_gain},

          {SERIAL_ACC_SET_OFFSET, &acc_set_offset},
          {SERIAL_ACC_SET_GAIN, &acc_set_gain},
          {SERIAL_ACC_GET_OFFSET, &acc_get_offset},
          {SERIAL_ACC_GET_GAIN, &acc_get_gain},

          {SERIAL_GYRO_SET_OFFSET, &gyro_set_offset},
          {SERIAL_GYRO_SET_GAIN, &gyro_set_gain},
          {SERIAL_GYRO_GET_OFFSET, &gyro_get_offset},
          {SERIAL_GYRO_GET_GAIN, &gyro_get_gain},

          {SERIAL_PRINT_NOTHING, &set_print_nothing},
          {SERIAL_PRINT_AHRS, &set_print_ahrs},
          {SERIAL_PRINT_MAG_RAW, &set_print_mag_raw},
          {SERIAL_PRINT_MAG_CALIB, &set_print_mag_calib},
          {SERIAL_PRINT_ACC_RAW, &set_print_acc_raw},
          {SERIAL_PRINT_ACC_CALIB, &set_print_acc_calib},
          {SERIAL_PRINT_GYRO_RAW, &set_print_gyro_raw},
          {SERIAL_PRINT_GYRO_CALIB, &set_print_gyro_calib} };

std::vector<std::string> split_input(std::string input, std::string delimiter) {
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

std::vector<float> split_and_strtof(std::string input, std::string delimiter) {
/*
  Split string, convert elements to float and return them as std::vector<float>.
*/
  std::vector<float> res;

  auto input_str_vec = split_input(input, ",");
  for (std::string str : input_str_vec) {
    res.emplace_back(strtof(str.c_str(), NULL));
  }

  return res;
}

void mag_set_offset(std::string input) {
  auto values = split_and_strtof(input, ",");

  MagOffset.x = values[0];
  MagOffset.y = values[1];
  MagOffset.z = values[2];
}

void mag_set_gain(std::string input) {
  auto values = split_and_strtof(input, ",");

  MagGain.x = values[1];
  MagGain.y = values[1];
  MagGain.z = values[2];
}

void mag_get_offset(std::string input) {
  std::string str = "Magnetic offset (x,y,z): " + MagOffset.to_string();
  Serial.println(str.c_str());
}

void mag_get_gain(std::string input) {
  std::string str = "Magnetic gain (x,y,z): " + MagGain.to_string();
  Serial.println(str.c_str());
}

void acc_set_offset(std::string input) {
  auto values = split_and_strtof(input, ",");

  AccOffset.x = values[0];
  AccOffset.y = values[1];
  AccOffset.z = values[2];
}

void acc_set_gain(std::string input) {
  auto values = split_and_strtof(input, ",");

  AccGain.x = values[0];
  AccGain.y = values[1];
  AccGain.z = values[2];
}

void acc_get_offset(std::string input) {
  std::string str = "Accelerometer offset (x,y,z): " + AccOffset.to_string();
  Serial.println(str.c_str());
}

void acc_get_gain(std::string input) {
  std::string str = "Accelerometer gain (x,y,z): " + AccGain.to_string();
  Serial.println(str.c_str());
}

void gyro_set_offset(std::string input) {
  auto values = split_and_strtof(input, ",");

  GyroOffset.x = values[0];
  GyroOffset.y = values[1];
  GyroOffset.z = values[2];
}

void gyro_set_gain(std::string input) {
  auto values = split_and_strtof(input, ",");

  GyroGain.x = values[0];
  GyroGain.y = values[1];
  GyroGain.z = values[2];
}

void gyro_get_offset(std::string input) {
  std::string str = "Gyroscope offset (x,y,z): " + GyroOffset.to_string();
  Serial.println(str.c_str());
}

void gyro_get_gain(std::string input) {
  std::string str = "Gyroscope gain (x,y,z): " + GyroGain.to_string();
  Serial.println(str.c_str());
}

void set_print_nothing(std::string input) {
  Serial.println("NOTHING");
  SerialOutputMode = SERIAL_PRINT_NOTHING;
}

void set_print_ahrs(std::string input) {
  Serial.println("AHRS");
  SerialOutputMode = SERIAL_PRINT_AHRS;
}

void set_print_mag_raw(std::string input) {
  SerialOutputMode = SERIAL_PRINT_MAG_RAW;
}
void set_print_mag_calib(std::string input) {
  SerialOutputMode = SERIAL_PRINT_MAG_CALIB;
}
void set_print_acc_raw(std::string input) {
  SerialOutputMode = SERIAL_PRINT_ACC_RAW;
}
void set_print_acc_calib(std::string input) {
  SerialOutputMode = SERIAL_PRINT_ACC_CALIB;
}
void set_print_gyro_raw(std::string input) {
  SerialOutputMode = SERIAL_PRINT_GYRO_RAW;
}
void set_print_gyro_calib(std::string input) {
  SerialOutputMode = SERIAL_PRINT_GYRO_CALIB;
}


void check_serial_input(void) {
/* 
  Check for commands in serial. 
*/
  static char serialBuffer[SERIAL_READ_BUFFER_SIZE];
  static const std::string OPTIONS_DELIMITER = ";";

  if (!Serial.available()) {
    return;
  }

  delay(50);  // Small delay in case the received message is 
              // incomplete when we check Serial.available. 
  Serial.println("Reading...");
  strncpy(serialBuffer, NULL, SERIAL_READ_BUFFER_SIZE);
  Serial.readBytesUntil('\n', serialBuffer, SERIAL_READ_BUFFER_SIZE);

  auto input_params = split_input(std::string(serialBuffer), OPTIONS_DELIMITER);
  // Guard against inputs which would crash the program at the "(it->second)(input_params[1])"
  if (input_params.size() == 1) input_params.emplace_back(" "); 

  uint8_t commandCode = static_cast<uint8_t>(strtol(input_params[0].c_str(), NULL, 16));
  auto it = input_functions.find(commandCode);
  if (it == input_functions.end()) {
    Serial.println("Command not found");
  } else {
    (it->second)(input_params[1]);
  }
}

/*
-------------------- END OF SERIAL INPUT PART --------------------
*/




void setup() {
    Serial.begin(SERIAL_BAUDRATE);

    IMU.setGyroscopeSettings( LSM9DS1_ODR_G_238HZ, LSM9DS1_FS_G_500DPS );
    IMU.setAccelerometerSettings( LSM9DS1_ODR_XL_119HZ, LSM9DS1_FS_XL_4G );
    IMU.begin(); // Start the STM LSM9DS1 inertial unit.

    // Madgwick fusion library initialization.
    FusionOffsetInitialise( &offset, SAMPLE_RATE );
    FusionAhrsInitialise( &AHRS );
    FusionAhrsSetSettings( &AHRS, &AHRSsettings );

    joystick.autoSend = false;
    joystick.sendBlocking = false;
    joystick.setXAxisRange( -180, 180 );  // left-right, yaw-axis. Range [-180, 180] degrees. 
    joystick.setYAxisRange( -90, 90 );    // up-down, pitch-axis. Range [-90, 90] degrees.
    joystick.setZAxisRange( -90, 90 );  // roll left-right, roll-axis. Range [-90, 90] degrees.

    //joystick.setXAxisRange( -90, 90 );  // left-right, yaw-axis. Range [-90, 90] degrees. 
    //joystick.setYAxisRange( -90, 90 );    // up-down, pitch-axis. Range [-90, 90] degrees.
    //joystick.setZAxisRange( -180, 180 );  // roll left-right, roll-axis. Range [-180, 180] degrees.

    IMU_previousTimeStamp = micros();

    Serial.println("System reset.");
}

void loop() {
  static uint32_t serial_output_timer = millis();
  static uint32_t serial_input_timer = millis();
  
  AHRS_check();

  if (millis() - serial_output_timer > 100) {
    serial_output_timer = millis();
    print_output();
  }

  if (millis() - serial_input_timer > 200) {
    serial_input_timer = millis();
    check_serial_input();
  }
}
