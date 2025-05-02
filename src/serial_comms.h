#ifndef __SERIAL_COMMS__
#define __SERIAL_COMMS__

#include <string>
#include <vector>

// TODO: Why are some constants as string and others as enum/int?

const std::string SERIAL_HANDSHAKE = "0x05";
const std::string SERIAL_DONE = "0x06";

enum {
  SERIAL_PRINT_NOTHING = 0x10,
  SERIAL_PRINT_MAG_RAW = 0x11,
  SERIAL_PRINT_MAG_CALIB = 0x12,
  SERIAL_PRINT_ACC_RAW = 0x13,
  SERIAL_PRINT_ACC_CALIB = 0x14,
  SERIAL_PRINT_GYRO_RAW = 0x15,
  SERIAL_PRINT_GYRO_CALIB = 0x16,
  SERIAL_PRINT_AHRS = 0x20,

  SERIAL_MAG_SET_CALIB = 0x30,
  SERIAL_MAG_GET_CALIB = 0x31,
  SERIAL_ACC_SET_CALIB = 0x40,
  SERIAL_ACC_GET_CALIB = 0x41,
  SERIAL_GYRO_SET_CALIB = 0x50,
  SERIAL_GYRO_GET_CALIB = 0x51,

  SERIAL_RESET_FACTORY_DEFAULTS = 0x60,
  SERIAL_RESET_KVSTORE = 0x70,

  SERIAL_BAUDRATE = 57600,
  SERIAL_READ_BUFFER_SIZE = 1024,
};

float remap_yaw(float yaw, float d);

bool serial_handshake(void);
std::string read_serial_input(void);
void execute_command(std::vector<std::string> params);
void check_serial_input(void);

void printAHRSeuler(void);
void printNothing(void);
void printAccCalib(void);
void printMagCalib(void);
void printGyroCalib(void);
void printAccRaw(void);
void printMagRaw(void);
void printGyroRaw(void);
void print_output(void);

void mag_set_calib(std::string input);
void mag_get_calib(std::string input);
void acc_set_calib(std::string input);
void acc_get_calib(std::string input);
void gyro_set_calib(std::string input);
void gyro_get_calib(std::string input);

void set_print_nothing(std::string input);
void set_print_ahrs(std::string input);
void set_print_mag_raw(std::string input);
void set_print_mag_calib(std::string input);
void set_print_acc_raw(std::string input);
void set_print_acc_calib(std::string input);
void set_print_gyro_raw(std::string input);
void set_print_gyro_calib(std::string input);

void kv_store_reset(std::string input);

#endif
