
//#include <Arduino.h>
//#include <Arduino_LSM9DS1.h>
#include "LSM9DS1.h"
#include <Fusion.h>
#include <MyVector.h>
#include <USBJoystick.h>

//#define NO_MAGNETOMETER

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
    SERIAL_COMMAND_SIZE = 4
} ;
auto SerialOutputMode = SERIAL_PRINT_AHRS;


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

/*
 IMU measurement variables and calibration.
*/
using MyVector::vector;

// Variables for acceleration values and calibrations.
vector Acc; 
vector rawAcc;
vector CurrentAcc;
vector AccGain(1.00f, 1.00f, 1.00f);
vector AccOffset(0.0325f, 0.0306f, 0.01819f); 
const float ACC_INTEG = 0.90f;

// Variables for gyroscope values and calibrations.
vector Gyro;
vector rawGyro;
vector CurrentGyro;
vector GyroGain(1.125f, 1.125f, 1.125f); 
//vector GyroOffset(-0.59112f, -0.77606f, -0.240580);  // 15 Hz values
//vector GyroOffset(-0.58712f, -0.69506f, 0.09000);    // 60 Hz values
//vector GyroOffset(-0.40738f, -0.73831f, -0.058472);  // 119 Hz values
vector GyroOffset(-0.50019f, -0.68556f, 0.13808f);     // 238 Hz values
const float GYRO_INTEG = 0.90f;

// Variables for magnetometer values and calibrations.
vector Mag;
vector rawMag;
vector CurrentMag;
//vector MagGain(1.0f/42.119f, 1.0f/40.031f, 1.0f/42.930f);
//vector MagOffset(-11.660f, -10.046f, 1.094f);
//vector MagGain(1.0f/44.767f, 1.0f/44.074f, 1.0f/42.064f);
vector MagGain(1.0f, 1.0f, 1.0f);
vector MagOffset(-7.257f, 39.747f, -11.817f);
const float MAG_INTEG = 0.90f;

FusionVector readAcceleration(void) {
    /* Read and smooth the IMU-data. Uses leaky integrator smoothing. */
    IMU.readAcceleration( Acc.x, Acc.y, Acc.z );
    Acc = (Acc + AccOffset) * AccGain;
    Acc = changeAxisSign(Acc, -1, -1, 1);
    CurrentAcc = CurrentAcc*ACC_INTEG + Acc*(1-ACC_INTEG);

    return MyVector_to_FusionVector( CurrentAcc );
}

FusionVector readGyroscope(void) {
    IMU.readGyroscope( Gyro.x, Gyro.y, Gyro.z );
    Gyro = (Gyro + GyroOffset) * GyroGain;
    Gyro = changeAxisSign( Gyro, 1, 1, -1 );
    CurrentGyro = CurrentGyro*GYRO_INTEG + Gyro*(1-GYRO_INTEG);

    return MyVector_to_FusionVector( CurrentGyro );
}

FusionVector readMagneticField(vector* rawMag) {
    //IMU.readMagneticField( Mag.x, Mag.y, Mag.z );
    IMU.readMagneticField( rawMag->x, rawMag->y, rawMag->z );
    // Serial.println(rawMag->x);
    Mag = (*rawMag - MagOffset) * MagGain;
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



void printAHRSeuler(void) {
    const FusionEuler euler = FusionQuaternionToEuler( FusionAhrsGetQuaternion( &AHRS ) );

    Serial.print( String("Roll: ") + String(euler.angle.roll, 2) + String(", ") );
    Serial.print( String("Pitch: ") + String(euler.angle.pitch, 2) + String(", ") );
    Serial.print( String("Yaw: ") + String(euler.angle.yaw, 2) + String(", ") );
}

void printAHRSearth(void) {
    const FusionVector earth = FusionAhrsGetEarthAcceleration( &AHRS );

    Serial.print( String("X: ") + String(earth.axis.x, 2) + String(", ") ); 
    Serial.print( String("Y: ") + String(earth.axis.y, 2) + String(", ") ); 
    Serial.print( String("Z: ") + String(earth.axis.z, 2) + String(", ") ); 
}

void updateTimeStamp(void) {
    /* Calculate timestep and convert from microseconds to seconds. */
    static const float MICROSECONDS_TO_SECONDS = 1.0f / 1000000.0f;

    // Variables defined globally, bad...
    IMU_timeStamp = micros();
    deltaTime = static_cast<float>( (IMU_timeStamp - IMU_previousTimeStamp) * MICROSECONDS_TO_SECONDS ); 
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

uint32_t printTimer;


/*
  SETUP
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
    printTimer = millis();

    Serial.println("System reset.");
}

void loop() {

#ifndef NO_MAGNETOMETER
  /* If acceleration, gyroscopy and magnetometer data are ready. 
   Accelerometer and gyroscope run with higher sample rate, magnetometer with 20 Hz. */
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()) {
    updateTimeStamp(); // Updates deltaTime variable.

    accelerometer = readAcceleration();   
    magnetometer = readMagneticField(&rawMag);
    gyroscope = readGyroscope();

    // Compensate for the long-term gyroscope drift.
    gyroscope = FusionOffsetUpdate( &offset, gyroscope ); 

    // Run the AHRS-algorithm.
    FusionAhrsUpdate( &AHRS, gyroscope, accelerometer, magnetometer, deltaTime ); 

    updateJoystickAxes( &AHRS );
  }

  /* If acceleration and gyroscope data are ready. */
  else if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
#endif
#ifdef NO_MAGNETOMETER
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
#endif
    updateTimeStamp(); // Updates deltaTime variable.
    accelerometer = readAcceleration();   
    gyroscope = readGyroscope();
    
    // Compensate for the long-term gyroscope drift.
    gyroscope = FusionOffsetUpdate( &offset, gyroscope ); 

    // Run the AHRS-algorithm
    FusionAhrsUpdateNoMagnetometer( &AHRS, gyroscope, accelerometer, deltaTime ); 

    updateJoystickAxes( &AHRS );
  }


  if (millis() - printTimer > 100) {
    printTimer = millis();

    switch (SerialOutputMode) {
      case SERIAL_PRINT_AHRS:
        printAHRSeuler();
        Serial.print( String("Heading: ") );
        Serial.println( FusionCompassCalculateHeading(accelerometer, magnetometer), 3 );
        break;
      case SERIAL_PRINT_MAG_RAW:
        rawMag.printVector();
        break;
      case SERIAL_PRINT_MAG_CALIB:
        Serial.print("x: ");
        Serial.print(magnetometer.axis.x);
        Serial.print(", y: ");
        Serial.print(magnetometer.axis.y);
        Serial.print(", z: ");
        Serial.println(magnetometer.axis.z);
        break;
    }


    /* Check for commands in serial. */
    if (Serial.available()) {
      delay(50);  // Small delay in case the received message is 
                  // incomplete when we check Serial.available. 

      char serialBuffer[SERIAL_READ_BUFFER_SIZE];
      Serial.readBytes(serialBuffer, SERIAL_COMMAND_SIZE);

      auto commandCode = strtol(serialBuffer, NULL, 16);
      switch (commandCode) {
        case SERIAL_PRINT_NOTHING:
          SerialOutputMode = SERIAL_PRINT_NOTHING;
          break;

        case SERIAL_PRINT_MAG_RAW:
          SerialOutputMode = SERIAL_PRINT_MAG_RAW;
          break;

        case SERIAL_PRINT_MAG_CALIB:
          SerialOutputMode = SERIAL_PRINT_MAG_CALIB;
          break;

        case SERIAL_PRINT_AHRS:
          SerialOutputMode = SERIAL_PRINT_AHRS;
          break;

        case SERIAL_MAG_SET_OFFSET:
          SerialOutputMode = SERIAL_PRINT_NOTHING;
          MagOffset.x = Serial.parseFloat();
          MagOffset.y = Serial.parseFloat();
          MagOffset.z = Serial.parseFloat();
          break;

        case SERIAL_MAG_SET_GAIN:
          SerialOutputMode = SERIAL_PRINT_NOTHING;
          MagGain.x = 1.0f/Serial.parseFloat();
          MagGain.y = 1.0f/Serial.parseFloat();
          MagGain.z = 1.0f/Serial.parseFloat();
          break;

        case SERIAL_MAG_GET_OFFSET:
          SerialOutputMode = SERIAL_PRINT_NOTHING;
          Serial.print("x: ");
          Serial.print(MagOffset.x);
          Serial.print(", y: ");
          Serial.print(MagOffset.y);
          Serial.print(", z: ");
          Serial.println(MagOffset.z);
          break;

        case SERIAL_MAG_GET_GAIN:
          SerialOutputMode = SERIAL_PRINT_NOTHING;
          Serial.print(", x: ");
          Serial.print(1.0f/MagGain.x);
          Serial.print(", y: ");
          Serial.print(1.0f/MagGain.y);
          Serial.print(", z: ");
          Serial.println(1.0f/MagGain.z);
          break;
      }

      // Clear waiting data from serial.
      while (Serial.available()) { 
          Serial.readBytes(serialBuffer, 1);
      }
    }
  }
}


