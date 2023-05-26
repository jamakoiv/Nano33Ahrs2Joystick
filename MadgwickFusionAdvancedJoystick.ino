
//#include <Arduino.h>
//#include <Arduino_LSM9DS1.h>
#include "LSM9DS1.h"
#include <Fusion.h>
#include <MyVector.h>
#include <USBJoystick.h>

//#define NO_MAGNETOMETER

USBJoystick joystick;



// Fusion-library objects and variables.
const uint32_t SAMPLE_RATE = 238;
FusionOffset offset;
FusionAhrs AHRS;


FusionVector gyroscope;
FusionVector accelerometer;
FusionVector magnetometer; 

// Set AHRS algorithm settings
const FusionAhrsSettings AHRSsettings = {
        .gain = 0.50f,
        .accelerationRejection = 10.0f,
        .magneticRejection = 20.0f,
        .rejectionTimeout = 2*SAMPLE_RATE
        //.rejectionTimeout = 0
} ;

uint32_t IMU_timeStamp, IMU_previousTimeStamp;
float deltaTime;


// IMU measurement variables and calibration.
using MyVector::vector;

vector Acc; 
vector CurrentAcc;
vector AccGain(1.00f, 1.00f, 1.00f);
vector AccOffset(0.0325f, 0.0306f, 0.01819f);
const float ACC_INTEG = 0.90f;

vector Gyro;
vector CurrentGyro;
vector GyroGain(1.125f, 1.125f, 1.125f); 
//vector GyroOffset(-0.59112f, -0.77606f, -0.240580);  // 15 Hz values
//vector GyroOffset(-0.58712f, -0.69506f, 0.09000);    // 60 Hz values
//vector GyroOffset(-0.40738f, -0.73831f, -0.058472);  // 119 Hz values
vector GyroOffset(-0.50019f, -0.68556f, 0.13808f);     // 238 Hz values
const float GYRO_INTEG = 0.90f;

vector Mag;
vector CurrentMag;
//vector MagGain(1.0f/42.119f, 1.0f/40.031f, 1.0f/42.930f);
//vector MagOffset(-11.660f, -10.046f, 1.094f);
vector MagGain(1.0f/44.767f, 1.0f/44.074f, 1.0f/42.064f);
vector MagOffset(-7.257f, 39.747f, -11.817f);
const float MAG_INTEG = 0.90f;


/*
  Read and smooth the IMU-data. Uses leaky integrator smoothing.
*/
FusionVector readAcceleration(void) {
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

FusionVector readMagneticField(void) {
    IMU.readMagneticField( Mag.x, Mag.y, Mag.z );
    Mag = (Mag + MagOffset) * MagGain;
    Mag = changeAxisSign( Mag, -1, 1, -1 );
    CurrentMag = CurrentMag*MAG_INTEG + Mag*(1-MAG_INTEG);

    return MyVector_to_FusionVector( CurrentMag );
}

/*
 Convert from MyVector::vector to FusionVector-object.
*/
FusionVector MyVector_to_FusionVector(const vector& vec) {
  FusionVector res;

  res.axis.x = vec.x;
  res.axis.y = vec.y;
  res.axis.z = vec.z;

  return res;
}

/*
  Change the axis-signs.
*/
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

// Calculate timestep and convert from microseconds to seconds.
void updateTimeStamp(void) {
  static const float MICROSECONDS_TO_SECONDS = 1.0f / 1000000.0f;

  // Variables defined globally, bad...
  IMU_timeStamp = micros();
  deltaTime = static_cast<float>( (IMU_timeStamp - IMU_previousTimeStamp) * MICROSECONDS_TO_SECONDS ); 
  IMU_previousTimeStamp = IMU_timeStamp;
}

/*
 Update the joystick-axes using the AHRS angle data.
*/
void updateJoystickAxes(const FusionAhrs *const ahrs) {
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

  Serial.begin(57600);

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
  /* 
   If acceleration, gyroscopy and magnetometer data are ready. 
   Accelerometer and gyroscope run with 120 Hz sample rate, magnetometer with 20 Hz.
  */
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()) {
    updateTimeStamp(); // Updates deltaTime variable.

    accelerometer = readAcceleration();   
    magnetometer = readMagneticField();
    gyroscope = readGyroscope();

    // Compensate for the long-term gyroscope drift.
    gyroscope = FusionOffsetUpdate( &offset, gyroscope ); 

    // Run the AHRS-algorithm.
    FusionAhrsUpdate( &AHRS, gyroscope, accelerometer, magnetometer, deltaTime ); 

    updateJoystickAxes( &AHRS );
  }

  /* 
    If acceleration and gyroscope data are ready.
  */
  else if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
#endif
#ifdef NO_MAGNETOMETER
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
#endif
    updateTimeStamp(); // Updates deltaTime variable.

    accelerometer = readAcceleration();   
    gyroscope = readGyroscope();
    gyroscope = FusionOffsetUpdate( &offset, gyroscope ); // Compensate for the long-term gyroscope drift.

    // Run the AHRS-algorithm
    FusionAhrsUpdateNoMagnetometer( &AHRS, gyroscope, accelerometer, deltaTime ); 

    updateJoystickAxes( &AHRS );
  }


  if (millis() - printTimer > 100) {
    printTimer = millis();

    printAHRSeuler();
    //printAHRSearth();

    float heading = FusionCompassCalculateHeading( accelerometer, magnetometer );
    Serial.print( String("Heading: ") );
    Serial.println( heading, 3);

    //CurrentGyro.printVector();
    //CurrentAcc.printVector();
    //CurrentMag.printVector();

    Gyro.printVector();
    //Mag.printVector();
    //CurrentMag.printVector();
    //MagGain.printVector();
  }
}


