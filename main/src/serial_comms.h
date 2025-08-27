#ifndef __SERIAL_COMMS__
#define __SERIAL_COMMS__

#include <stdint.h>
#include <string>
#include <vector>

#include "serial_utils.h"

using std::string;
using std::vector;

enum {
    SERIAL_SET_PRINT_MODE = 0x10,
    SERIAL_PRINT_NOTHING = 0x15,
    SERIAL_PRINT_AHRS = 0x16,
    SERIAL_PRINT_AHRS_DEBUG = 0x17,

    SERIAL_MAG_SET_CALIB = 0x30,
    SERIAL_MAG_GET_CALIB = 0x31,
    SERIAL_PRINT_MAG_RAW = 0x35,
    SERIAL_PRINT_MAG_CALIB = 0x36,

    SERIAL_ACC_SET_CALIB = 0x40,
    SERIAL_ACC_GET_CALIB = 0x41,
    SERIAL_PRINT_ACC_RAW = 0x45,
    SERIAL_PRINT_ACC_CALIB = 0x46,

    SERIAL_GYRO_SET_CALIB = 0x50,
    SERIAL_GYRO_GET_CALIB = 0x51,
    SERIAL_PRINT_GYRO_RAW = 0x55,
    SERIAL_PRINT_GYRO_CALIB = 0x56,

    SERIAL_SET_OFFSET = 0x80,
    SERIAL_GET_OFFSET = 0x81,

    SERIAL_RESET_KVSTORE = 0x70,
};

static const int SERIAL_BAUDRATE =
    57600; // Not actually used when using USB-serial
static const int SERIAL_READ_BUFFER_SIZE = 2048; // 2 kB
static char serialBuffer[SERIAL_READ_BUFFER_SIZE];

string execute_command(command_t &commands);

string print_output(void);
string set_print_mode(vector<float> params);

string mag_set_calib(vector<float> params);
string acc_set_calib(vector<float> params);
string gyro_set_calib(vector<float> params);
string yaw_set_offset(vector<float> params);

string acc_get_calib(void);
string mag_get_calib(void);
string gyro_get_calib(void);
string yaw_get_offset(void);

string _kv_store_reset(void);

#endif
