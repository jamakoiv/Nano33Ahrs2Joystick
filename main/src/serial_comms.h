#ifndef __SERIAL_COMMS__
#define __SERIAL_COMMS__

#include <stdint.h>
#include <string>
#include <vector>

#include "serial_utils.h"

enum {
    SERIAL_SET_PRINT_MODE = 0x10,
    SERIAL_PRINT_NOTHING = 0x15,
    SERIAL_PRINT_AHRS = 0x16,
    SERIAL_PRINT_AHRS_DEBUG = 0x17,

    SERIAL_MAG_SET_CALIB = 0x30,
    SERIAL_MAG_GET_CALIB = 0x31,
    SERIAL_PRINT_MAG_RAW = 0x35,
    SERIAL_PRINT_MAG_CALIB = 0x36,
    SERIAL_PRINT_MAG_GYRO_RAW = 0x37,

    SERIAL_ACC_SET_CALIB = 0x40,
    SERIAL_ACC_GET_CALIB = 0x41,
    SERIAL_PRINT_ACC_RAW = 0x45,
    SERIAL_PRINT_ACC_CALIB = 0x46,

    SERIAL_GYRO_SET_CALIB = 0x50,
    SERIAL_GYRO_GET_CALIB = 0x51,
    SERIAL_PRINT_GYRO_RAW = 0x55,
    SERIAL_PRINT_GYRO_CALIB = 0x56,

    SERIAL_MISC_SET_SETTINGS = 0x60,
    SERIAL_MISC_GET_SETTINGS = 0x61,

    SERIAL_SET_OFFSET = 0x70,
    SERIAL_GET_OFFSET = 0x71,

    SERIAL_RESET_KVSTORE = 0x75,
};

std::string execute_command(command_t &commands);

std::string print_output(void);
std::string set_print_mode(std::vector<float> params);

std::string mag_set_calib(std::vector<float> params);
std::string acc_set_calib(std::vector<float> params);
std::string gyro_set_calib(std::vector<float> params);
std::string yaw_set_offset(std::vector<float> params);

std::string acc_get_calib(void);
std::string mag_get_calib(void);
std::string gyro_get_calib(void);
std::string yaw_get_offset(void);

std::string _kv_store_reset(void);

#endif
