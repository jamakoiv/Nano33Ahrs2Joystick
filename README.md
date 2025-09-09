Calculates the orientation of the board from the IMU sensor data using Madgwick AHRS -filter.
The current board orientation is sent to the host computer as USB-joystick axes.

Raw or calibrated sensor values can be read using serial-output from the board, and persistent calibration
values can be set also via serial-input.

Currently calculation of calibration values is performed on the host computer using separate [program](https://github.com/jamakoiv/nano33_magnetic_calibration).
