/*
 * Copyright (c) 2022, Jaakko Koivisto.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// TODO: Change the filename and class-name to something more expressive now
// that the joystick-related stuff is handled in the Joystick-class.

#ifndef __USBJOYSTICK_H__
#define __USBJOYSTICK_H__

#include "Joystick.h"
#include "PlatformMutex.h"
#include "PluggableUSBHID.h"

#include <cstdint>
#include <tuple>

namespace arduino {

class USBCommsJoystick : public USBHID {
private:
  bool sendBlocking = false;
  bool autoSend = false;

  // TODO: Change BUTTONS_MAX_NUMBER to be set by user.
  static const uint8_t BUTTONS_MAX_NUMBER = 64;

  // Absolute maximum number of buttons 32*8 = 256.
  static const uint8_t BUTTON_ARRAY_MAX_SIZE = 32;

  // How many bits is in a single byte of data.
  static const uint8_t BYTE_LENGTH = 8;

  // Axis data will be constrained to this range when sending it over the USB.
  // TODO: 16-bit resolution works on windows but not on linux. Maybe something
  // wrong with the HID-report?
  static const int16_t HID_AXIS_MIN = -2047; // 12-bit
  static const int16_t HID_AXIS_MAX = 2047;

  // TODO:  Report IDs have to be declared static or otherwise the
  // report-descriptor becomes corrupted and the whole USB-device goes down.
  static const uint8_t REPORT_ID = 0x10;

  // TODO: Get rid of hardcoded magic numbers.
  uint8_t _configuration_descriptor[34]; // 34 bytes.

  HID_REPORT HIDreport;

  /*  INFO: !!!WARNING!!! Do not move the PlatformMutex! Putting it above the
        other variables leads to some very strange bugs where accessing the
     variables inside method 'update' accesses different memory-address than
     same variable from other methods.
  */
  PlatformMutex _mutex;

public:
  static const bool SEND_BLOCKING = true;
  static const bool SEND_NONBLOCKING = false;
  static const bool AUTOSEND = true;
  static const bool NO_AUTOSEND = false;

  /*
    Constuctors and destructors.
  */
  USBCommsJoystick(bool connect_blocking = true, uint16_t vendor_id = 0x1235,
                   uint16_t product_id = 0x0050,
                   uint16_t product_release = 0x0001);

  USBCommsJoystick(USBPhy *phy, uint16_t vendor_id = 0x1235,
                   uint16_t product_id = 0x0050,
                   uint16_t product_release = 0x0001);

  virtual ~USBCommsJoystick(void);

  /*
   Construct the HID-report descriptor.

   @returns pointer to the report descriptor.
  */
  // TODO: Who calls this function and when???
  // Probably when the class is constructed and the USB-system goes through with
  // the regular init-hoops. Check USBHID and underlying classes, maybe we find
  // something...
  virtual const uint8_t *report_desc(void);

  // TODO: Do we need these? 'report_rx' is used only when Host sends HID-report
  // to the Device. Do we need to use 'report_tx' or simply use 'send'
  // periodically???
  // virtual void report_rx(void); // Called when there is a hid report that can
  // be read. virtual void report_tx(void); // Called when there is space to
  // send a hid report (space in the USB-bus??)

  /*
    Send a USB HID-report to the host. 'send' for blocking and 'send_nb' for
    non-blocking. These are inherited from the USBHID class.
  */
  // bool send(HID_REPORT report);
  // bool send_nb(HID_REPORT report);

  /*
    Read a USB HID-report from the host. 'read' for blocking and 'read_nb' for
    non-blocking. These are inherited from the USBHID class.
  */
  // bool read(HID_REPORT report);
  // bool read_nb(HID_REPORT report);

  /*
    Send joystick state update to the host.

    @returns true if there was no error, false otherwise. Return values passed
    through from 'USBHID::send'.
  */
  bool update();

  /*
    Wrapper. For API-compliance with the MHeironimus-ArduinoJoystickLibrary.
  */
  void sendState(void) { this->update(); }

  /*
    These don't really do much currently. For API-compliance with the
    MHeironimus-ArduinoJoystickLibrary.
  */
  void begin(bool autoSendState) { this->autoSend = autoSendState; }
  void end(void) {}

  /*
    Update the HID-report with the current joystick-state (axis-values,
    button-states etc.).
  */
  void updateHIDreport(const Joystick *const joystick);

  /*
    Get the lower (LSB) or higher (MSB) 8-bits of 16-bit axis-value.
    You have to call this twice with MSB and LSB to get the full axis value.
    Used to store the axis value in the HID-report which is made of 8-bit
    fields.

    @returns 8-bit integer containing the least-significant or most significant
    byte.
  */
  int8_t axis16bitToByte(int16_t axisValue, bool MSB_OR_LSB);

  /*
    Template implementation of Arduino map-function for arbitary input and
    output data-types.
  */
  // template<typename fromType, typename toType>
  // static toType map(fromType x, fromType in_min, fromType in_max, toType
  // out_min, toType out_max)
  //{
  //   return (x -in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  // }

  /*
    Maps value in the range [in_min:in_max] to [out_min:out_max].
    Local implementation of Arduino map-function for floating-point data input.
    Takes input as 'float' and output as 'int16_t'.

    @returns mapped value as 16-bit integer.
  */
  static inline int16_t mapfi(float x, float in_min, float in_max,
                              int16_t out_min, int16_t out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  void setSettings(bool autoSend, bool sendBlocking);
  std::tuple<bool, bool> getSettings(void) const;
  std::tuple<const bool *const, const bool *> getSettingsPtr(void) const;

protected:
  /*
   Get configuration descriptor.

   @returns pointer to the configuration descriptor.
  */
  // TODO: Who calls this function? Something deep inside the USB-stack???
  virtual const uint8_t *configuration_desc(uint8_t index);

}; // class USBCommsJoystick

} // namespace arduino

#endif // USBJOYSTICK_H
