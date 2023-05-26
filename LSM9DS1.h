/*
  This file is part of the Arduino_LSM9DS1 library.
  Copyright (c) 2019 Arduino SA. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <Arduino.h>
#include <Wire.h>


// Gyroscope output data rate (ODR) and full-scale (FS) settings.
#define LSM9DS1_ODR_G_POWERDOWN   0x00
#define LSM9DS1_ODR_G_15HZ        0x20
#define LSM9DS1_ODR_G_60HZ        0x40
#define LSM9DS1_ODR_G_119HZ       0x60
#define LSM9DS1_ODR_G_238HZ       0x80
#define LSM9DS1_ODR_G_476HZ       0xa0
#define LSM9DS1_ODR_G_952HZ       0xc0

#define LSM9DS1_FS_G_245DPS       0x00
#define LSM9DS1_FS_G_500DPS       0x08
#define LSM9DS1_FS_G_2000DPS      0x18

#define LSM9DS1_BW_G_00           0x00
#define LSM9DS1_BW_G_01           0x01
#define LSM9DS1_BW_G_10           0x02
#define LSM9DS1_BW_G_11           0x03


// Accelerometer output data rate (ODR), scale (FS) and bandwidth (BW) settings.
#define LSM9DS1_ODR_XL_POWERDOWN  0x00
#define LSM9DS1_ODR_XL_10HZ       0x20
#define LSM9DS1_ODR_XL_50HZ       0x40
#define LSM9DS1_ODR_XL_119HZ      0x60
#define LSM9DS1_ODR_XL_238HZ      0x80
#define LSM9DS1_ODR_XL_476HZ      0xa0
#define LSM9DS1_ODR_XL_952HZ      0xc0

#define LSM9DS1_FS_XL_2G          0x00
#define LSM9DS1_FS_XL_4G          0x10
#define LSM9DS1_FS_XL_8G          0x18
#define LSM9DS1_FS_XL_16G         0x08

#define LSM9DS1_BW_SCAL_DETERMINED_BY_ODR   0x00 
#define LSM9DS1_BW_SCAL_DETERMINED_BY_BWXL  0x04

#define LSM9DS1_BW_XL_408HZ       0x00
#define LSM9DS1_BW_XL_211HZ       0x01
#define LSM9DS1_BW_XL_105HZ       0x02
#define LSM9DS1_BW_XL_50HZ        0x03


class LSM9DS1Class {

  struct gyroscopeSettingsStruct {
    int hex;
    float ODR;
    float FS;
    int BW;
  } ;

  struct accelerometerSettingsStruct {
    int hex;
    float ODR;
    float FS;
    bool BW_SCAL;
    int BW;
  } ;

  public:
    LSM9DS1Class(TwoWire& wire);
    virtual ~LSM9DS1Class();

    int begin();
    void end();

    // Controls whether a FIFO is continuously filled, or a single reading is stored.
    // Defaults to one-shot.
    void setContinuousMode();
    void setOneShotMode();

    void setGyroscopeSettings(int outputDataRate=LSM9DS1_ODR_G_119HZ,
                              int fullScale=LSM9DS1_FS_G_2000DPS, 
                              int bandWidth=LSM9DS1_BW_G_00);

    void setAccelerometerSettings(int outputDataRate=LSM9DS1_ODR_XL_119HZ, 
                                  int fullScale=LSM9DS1_FS_XL_4G, 
                                  int bandWidthSelect=LSM9DS1_BW_SCAL_DETERMINED_BY_ODR, 
                                  int bandWidth=LSM9DS1_BW_XL_408HZ);

    // Accelerometer
    virtual int readAcceleration(float& x, float& y, float& z); // Results are in G (earth gravity).
    virtual int accelerationAvailable(); // Number of samples in the FIFO.
    virtual float accelerationSampleRate(); // Sampling rate of the sensor.

    // Gyroscope
    virtual int readGyroscope(float& x, float& y, float& z); // Results are in degrees/second.
    virtual int gyroscopeAvailable(); // Number of samples in the FIFO.
    virtual float gyroscopeSampleRate(); // Sampling rate of the sensor.

    // Magnetometer
    virtual int readMagneticField(float& x, float& y, float& z); // Results are in uT (micro Tesla).
    virtual int magneticFieldAvailable(); // Number of samples in the FIFO.
    virtual float magneticFieldSampleRate(); // Sampling rate of the sensor.

  private:
    bool continuousMode;
    int readRegister(uint8_t slaveAddress, uint8_t address);
    int readRegisters(uint8_t slaveAddress, uint8_t address, uint8_t* data, size_t length);
    int writeRegister(uint8_t slaveAddress, uint8_t address, uint8_t value);


  private:
    TwoWire* _wire;
    gyroscopeSettingsStruct _gyroscopeSettings;
    accelerometerSettingsStruct _accelerometerSettings;
};

extern LSM9DS1Class IMU;
