#include "Joystick.h"
#include <Arduino.h>
#include <algorithm>
#include <cstdint>

namespace arduino {
// TODO: Cut & paste of same code in 'pressButton', 'releaseButton' and '
// toggleButton'.
void Joystick::pressButton(uint8_t buttonNumber) {
  // buttonState is array of 8-bit integers where each bit represents current
  // button state. buttonState[0] has buttons 0-7, buttonState[1] has buttons
  // 8-15 etc.

  // Get the array index and bit position for the button number 'buttonNumber'.
  uint8_t index = buttonNumber / this->BYTE_LENGTH;
  uint8_t position = buttonNumber % this->BYTE_LENGTH;

  bitSet(this->buttonState[index], position);
}

void Joystick::releaseButton(uint8_t buttonNumber) {
  // See 'USBJoystick::pressButton for explanation of 'index' and 'position'.
  uint8_t index = buttonNumber / this->BYTE_LENGTH;
  uint8_t position = buttonNumber % this->BYTE_LENGTH;

  bitClear(this->buttonState[index], position);
}

void Joystick::toggleButton(uint8_t buttonNumber) {
  // See 'USBJoystick::pressButton for explanation of 'index' and 'position'.
  uint8_t index = buttonNumber / this->BYTE_LENGTH;
  uint8_t position = buttonNumber % this->BYTE_LENGTH;

  this->buttonState[index] ^= 0x01 << position; // XOR to toggle a bit.
}

void Joystick::setButton(uint8_t buttonNumber, uint8_t value) {
  if (value == 0)
    this->releaseButton(buttonNumber);
  else
    this->pressButton(buttonNumber);
}

void Joystick::setAxis(float value, int AXIS) {
  switch (AXIS) {
  case X:
    this->axis.X = std::clamp(value, this->axisMin.X, this->axisMax.X);
    break;
  case Y:
    this->axis.Y = std::clamp(value, this->axisMin.Y, this->axisMax.Y);
    break;
  case Z:
    this->axis.Z = std::clamp(value, this->axisMin.Z, this->axisMax.Z);
    break;
  case Rx:
    this->axis.Rx = std::clamp(value, this->axisMin.Rx, this->axisMax.Rx);
    break;
  case Ry:
    this->axis.Ry = std::clamp(value, this->axisMin.Ry, this->axisMax.Ry);
    break;
  case Rz:
    this->axis.Rz = std::clamp(value, this->axisMin.Rz, this->axisMax.Rz);
    break;
  case slider0:
    this->axis.slider0 =
        std::clamp(value, this->axisMin.slider0, this->axisMax.slider0);
    break;
  case slider1:
    this->axis.Rz =
        std::clamp(value, this->axisMin.slider1, this->axisMax.slider1);
    break;
  }
}

void Joystick::setAxisRange(float minimum, float maximum, int AXIS) {
  switch (AXIS) {
  case X:
    this->axisMin.X = std::min(minimum, maximum);
    this->axisMax.X = std::max(minimum, maximum);
    break;
  case Y:
    this->axisMin.Y = std::min(minimum, maximum);
    this->axisMax.Y = std::max(minimum, maximum);
    break;
  case Z:
    this->axisMin.Z = std::min(minimum, maximum);
    this->axisMax.Z = std::max(minimum, maximum);
    break;
  case Rx:
    this->axisMin.Rx = std::min(minimum, maximum);
    this->axisMax.Rx = std::max(minimum, maximum);
    break;
  case Ry:
    this->axisMin.Ry = std::min(minimum, maximum);
    this->axisMax.Ry = std::max(minimum, maximum);
    break;
  case Rz:
    this->axisMin.Rz = std::min(minimum, maximum);
    this->axisMax.Rz = std::max(minimum, maximum);
    break;
  case slider0:
    this->axisMin.slider0 = std::min(minimum, maximum);
    this->axisMax.slider0 = std::max(minimum, maximum);
    break;
  case slider1:
    this->axisMin.slider1 = std::min(minimum, maximum);
    this->axisMax.slider1 = std::max(minimum, maximum);
    break;
  }
}

void Joystick::setAllAxisRange(float minimum, float maximum) {
  this->setAxisRange(minimum, maximum, X);
  this->setAxisRange(minimum, maximum, Y);
  this->setAxisRange(minimum, maximum, Z);
  this->setAxisRange(minimum, maximum, Rx);
  this->setAxisRange(minimum, maximum, Ry);
  this->setAxisRange(minimum, maximum, Rz);
  this->setAxisRange(minimum, maximum, slider0);
  this->setAxisRange(minimum, maximum, slider1);
}
} // namespace arduino
