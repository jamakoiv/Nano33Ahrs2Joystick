#include "Joystick.h"
#include <Arduino.h>
#include <algorithm>
#include <cstdint>

namespace arduino {

uint8_t Joystick::getButtonBytesAmount(void) const {
  return this->BUTTONS_MAX_NUMBER / this->BYTE_LENGTH;
}

uint8_t Joystick::getAxisAmount(void) const { return this->AXIS_AMOUNT; }

void Joystick::setAxis(float value, int AXIS) {
  this->axis.array[AXIS] =
      constrain(value, this->axisMin.array[AXIS], this->axisMax.array[AXIS]);
}

void Joystick::setAxisRange(float minimum, float maximum, int AXIS) {
  this->axisMin.array[AXIS] = std::min(minimum, maximum);
  this->axisMax.array[AXIS] = std::max(minimum, maximum);
}

float Joystick::getAxis(int AXIS) const { return this->axis.array[AXIS]; }

float Joystick::getAxisMin(int AXIS) const { return this->axisMin.array[AXIS]; }

float Joystick::getAxisMax(int AXIS) const { return this->axisMax.array[AXIS]; }

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
  // See 'USBCommsJoystick::pressButton for explanation of 'index' and
  // 'position'.
  uint8_t index = buttonNumber / this->BYTE_LENGTH;
  uint8_t position = buttonNumber % this->BYTE_LENGTH;

  bitClear(this->buttonState[index], position);
}

void Joystick::toggleButton(uint8_t buttonNumber) {
  // See 'USBCommsJoystick::pressButton for explanation of 'index' and
  // 'position'.
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

} // namespace arduino
