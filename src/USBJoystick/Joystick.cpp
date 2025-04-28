#include "Joystick.h"
#include <Arduino.h>
#include <algorithm>
#include <cstdint>

namespace arduino {

uint8_t Joystick::getButtonBytesAmount(void) {
  return this->BUTTONS_MAX_NUMBER / this->BYTE_LENGTH;
}

uint8_t Joystick::getAxisAmount(void) { return this->AXIS_AMOUNT; }

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
    this->axis.name.X =
        constrain(value, this->axisMin.name.X, this->axisMax.name.X);
    break;
  case Y:
    this->axis.name.Y =
        constrain(value, this->axisMin.name.Y, this->axisMax.name.Y);
    break;
  case Z:
    this->axis.name.Z =
        constrain(value, this->axisMin.name.Z, this->axisMax.name.Z);
    break;
  case Rx:
    this->axis.name.Rx =
        constrain(value, this->axisMin.name.Rx, this->axisMax.name.Rx);
    break;
  case Ry:
    this->axis.name.Ry =
        constrain(value, this->axisMin.name.Ry, this->axisMax.name.Ry);
    break;
  case Rz:
    this->axis.name.Rz =
        constrain(value, this->axisMin.name.Rz, this->axisMax.name.Rz);
    break;
  case slider0:
    this->axis.name.slider0 = constrain(value, this->axisMin.name.slider0,
                                        this->axisMax.name.slider0);
    break;
  case slider1:
    this->axis.name.Rz = constrain(value, this->axisMin.name.slider1,
                                   this->axisMax.name.slider1);
    break;
  }
}

void Joystick::setAxisRange(float minimum, float maximum, int AXIS) {
  switch (AXIS) {
  case X:
    this->axisMin.name.X = std::min(minimum, maximum);
    this->axisMax.name.X = std::max(minimum, maximum);
    break;
  case Y:
    this->axisMin.name.Y = std::min(minimum, maximum);
    this->axisMax.name.Y = std::max(minimum, maximum);
    break;
  case Z:
    this->axisMin.name.Z = std::min(minimum, maximum);
    this->axisMax.name.Z = std::max(minimum, maximum);
    break;
  case Rx:
    this->axisMin.name.Rx = std::min(minimum, maximum);
    this->axisMax.name.Rx = std::max(minimum, maximum);
    break;
  case Ry:
    this->axisMin.name.Ry = std::min(minimum, maximum);
    this->axisMax.name.Ry = std::max(minimum, maximum);
    break;
  case Rz:
    this->axisMin.name.Rz = std::min(minimum, maximum);
    this->axisMax.name.Rz = std::max(minimum, maximum);
    break;
  case slider0:
    this->axisMin.name.slider0 = std::min(minimum, maximum);
    this->axisMax.name.slider0 = std::max(minimum, maximum);
    break;
  case slider1:
    this->axisMin.name.slider1 = std::min(minimum, maximum);
    this->axisMax.name.slider1 = std::max(minimum, maximum);
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
