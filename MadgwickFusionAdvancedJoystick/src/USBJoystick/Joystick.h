#ifndef __JOYSTICK_H__
#define __JOYSTICK_H__

#include <cstdint>
#include <tuple>

namespace arduino {

typedef union {
  float array[8];

  struct axis_t {
    float X;
    float Y;
    float Z;
    float Rx;
    float Ry;
    float Rz;
    float slider0;
    float slider1;

  } name;
} axis_t;

enum { X, Y, Z, Rx, Ry, Rz, slider0, slider1 };

enum { MSB, LSB };

class Joystick {
private:
  static const uint8_t BYTE_LENGTH = 8;

  // Actual max amount of buttons we are using.
  static const uint8_t BUTTONS_MAX_NUMBER = 64;

  // slider0 and slider1 are currently not used.
  static const uint8_t AXIS_AMOUNT = 6;

  // TODO: Replace MBED_ASSERTS with c++ assert.
  // Buttons amount must be byte-aligned because I don't want to deal with
  // padding-data in the HID-report :)
  // MBED_STATIC_ASSERT(BUTTONS_MAX_NUMBER % BYTE_LENGTH == 0,
  //                    "BUTTONS_MAX_NUMBER does not align to byte-length");

  axis_t axis;
  axis_t axisMin;
  axis_t axisMax;

public:
  uint8_t buttonState[BUTTONS_MAX_NUMBER];

  Joystick() = default;
  ~Joystick() = default;

  void setAxis(float value, int AXIS);
  void setAxisRange(float minimum, float maximum, int AXIS);
  void setAllAxisRange(float minimum, float maximum);

  uint8_t getButtonBytesAmount(void) const;
  uint8_t getAxisAmount(void) const;

  float getAxis(int AXIS) const;
  std::tuple<float, float> getAxisRange(int AXIS) const;

  /*
    Press button. Sets button 'buttonNumber' state to 1.
    Release button. Sets button 'buttonNumber' state to 0.
    Toggle button. Invert the current state of the button 'buttonNumber'.
    Set button 'buttonNumber' state to 0 if value is 0, 1 if value is non-zero.
  */
  void pressButton(uint8_t buttonNumber);
  void releaseButton(uint8_t buttonNumber);
  void toggleButton(uint8_t buttonNumber);
  void setButton(uint8_t buttonNumber, uint8_t value);
};

} // namespace arduino

#endif
