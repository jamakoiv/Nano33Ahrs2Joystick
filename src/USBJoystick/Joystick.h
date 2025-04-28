
#include <cstdint>

namespace arduino {

struct axis_t {
  float X;
  float Y;
  float Z;
  float Rx;
  float Ry;
  float Rz;
  float slider0;
  float slider1;
};

enum { X, Y, Z, Rx, Ry, Rz, slider0, slider1 };

enum { MSB, LSB };

class Joystick {
private:
  static const uint8_t BYTE_LENGTH = 8;
  static const uint8_t BUTTONS_MAX_NUMBER =
      64; // Actual max amount of buttons we are using.
  //
  uint8_t buttonState[BUTTONS_MAX_NUMBER];

  axis_t axis;
  axis_t axisMin;
  axis_t axisMax;

public:
  Joystick();
  ~Joystick();

  void setAxis(float value, int AXIS);
  void setAxisRange(float minimum, float maximum, int AXIS);

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
