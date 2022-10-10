// BOARD=arduino:avr:micro
// UPLOAD_PORT=/dev/ttyACM0
//
//

#include <Arduino.h>
#include <Arduino_LSM9DS1.h>
#include <Fusion.h>


#define SAMPLE_RATE (120)



// Define calibration (replace with actual calibration data)
const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
const FusionVector gyroscopeOffset = { 0.60f, 0.07f, -0.48f };
const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
const FusionVector accelerometerOffset = { -0.032f, -0.031f, -0.018f };
const FusionMatrix softIronMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector magneticFieldOffset = { 13.49f, 28.5275f, -9.5765f };

// Set AHRS algorithm settings
const FusionAhrsSettings AHRSsettings = {
        .gain = 2.5f,
        .accelerationRejection = 10.0f,
        .magneticRejection = 20.0f,
        //.rejectionTimeout = 5*SAMPLE_RATE, 
        .rejectionTimeout = 0, 
};

FusionOffset offset;
FusionAhrs AHRS;

FusionVector gyroscope;
FusionVector accelerometer;
FusionVector magnetometer; 

uint32_t IMU_timeStamp, IMU_previousTimeStamp;
float deltaTime;


void readIMU(void) {
  double tmpX, tmpY, tmpZ;
  //IMU.readAcceleration( accelerometer.axis.x, accelerometer.axis.y, accelerometer.axis.z );
  IMU.readAcceleration( tmpX, tmpY, tmpZ );
  accelerometer.axis.x = tmpX;
  accelerometer.axis.y = tmpY;
  accelerometer.axis.z = tmpZ;

  //IMU.readGyroscope( gyroscope.axis.x, gyroscope.axis.y, gyroscope.axis.z );
  IMU.readGyroscope( tmpX, tmpY, tmpZ );
  gyroscope.axis.x = tmpX;
  gyroscope.axis.y = tmpY;
  gyroscope.axis.z = tmpZ;

  //IMU.readMagneticField( magnetometer.axis.x, magnetometer.axis.y, magnetometer.axis.z );
  IMU.readMagneticField( tmpX, tmpY, tmpZ );
  magnetometer.axis.x = tmpX;
  magnetometer.axis.y = tmpY;
  magnetometer.axis.z = tmpZ;
}

void calibrateMeasurements(void) {
    gyroscope = FusionCalibrationInertial( gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset );
    accelerometer = FusionCalibrationInertial( accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset );

    magnetometer = FusionCalibrationMagnetic( magnetometer, softIronMatrix, magneticFieldOffset );

    accelerometer = FusionAxesSwap( accelerometer, FusionAxesAlignmentNXNYPZ );
    // gyroscope = FusionAxesSwap( gyroscope, FusionAxesAlignmentPXPYNZ ); // TODO: Does not work, WHY???
    gyroscope.axis.z *= -1;
    magnetometer = FusionAxesSwap( magnetometer, FusionAxesAlignmentNXPYNZ );

    gyroscope = FusionOffsetUpdate( &offset, gyroscope );
}

void printAHRSeuler(void) {
  const FusionEuler euler = FusionQuaternionToEuler( FusionAhrsGetQuaternion( &AHRS ) );

  Serial.print( String("Roll: ") + String(euler.angle.roll, 2) + String(", ") );
  Serial.print( String("Pitch: ") + String(euler.angle.pitch, 2) + String(", ") );
  Serial.println( String("Yaw: ") + String(euler.angle.yaw, 2) );
}

void printAHRSearth(void) {
  const FusionVector earth = FusionAhrsGetEarthAcceleration( &AHRS );

  Serial.print( String("X: ") + String(earth.axis.x, 2) + String(", ") ); 
  Serial.print( String("Y: ") + String(earth.axis.y, 2) + String(", ") ); 
  Serial.println( String("Z: ") + String(earth.axis.z, 2) ); 
}

// Calculate timestep and convert from microseconds to seconds.
void updateTimeStamp(void) {
  IMU_timeStamp = micros();
  deltaTime = (float) (IMU_timeStamp - IMU_previousTimeStamp) / 1000000.0f; 
  IMU_previousTimeStamp = IMU_timeStamp;
}

uint32_t printTimer;

void setup() {

  Serial.begin(57600);
  IMU.begin();

  FusionOffsetInitialise( &offset, SAMPLE_RATE );
  FusionAhrsInitialise( &AHRS );
  FusionAhrsSetSettings( &AHRS, &AHRSsettings );

  IMU_previousTimeStamp = micros();
  printTimer = millis();

  Serial.println("System reset.");
}

void loop() {

  
  //if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()) {
    // Accelerometer and gyroscope run with 120 Hz sample rate, magnetometer with 20 Hz.

    readIMU();
    updateTimeStamp();
    calibrateMeasurements();

    FusionAhrsUpdate( &AHRS, gyroscope, accelerometer, magnetometer, deltaTime );
    //FusionAhrsUpdateNoMagnetometer( &AHRS, gyroscope, accelerometer, deltaTime );
  }

  if (millis() - printTimer > 100) {
    printAHRSeuler();
    //printAHRSearth();
    //Serial.println(deltaTime, 6);
    //Serial.println(SAMPLE_RATE);

    float heading = FusionCompassCalculateHeading( accelerometer, magnetometer );

    //Serial.print(String("Ax: ") + String(accelerometer.axis.x, 2) + String(", "));
    //Serial.print(String("Ay: ") + String(accelerometer.axis.y, 2) + String(", "));
    //Serial.print(String("Az: ") + String(accelerometer.axis.z, 2) + String("; "));

    //Serial.print(String("Gx: ") + String(gyroscope.axis.x, 2) + String(", "));
    //Serial.print(String("Gy: ") + String(gyroscope.axis.y, 2) + String(", "));
    //Serial.print(String("Gz: ") + String(gyroscope.axis.z, 2) + String("; "));

    //Serial.print(String("Mx: ") + String(magnetometer.axis.x, 2) + String(", "));
    //Serial.print(String("My: ") + String(magnetometer.axis.y, 2) + String(", "));
    //Serial.println(String("Mz: ") + String(magnetometer.axis.z, 2) + String("; "));

    Serial.println( heading, 2 );


    printTimer = millis();
  }
}


