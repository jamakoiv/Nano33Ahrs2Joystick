#ifndef __SERIAL_COMMS__
#define __SERIAL_COMMS__

#include <stdint.h>
#include <string>
#include <vector>

enum {
    SERIAL_START = 0x0A,
    SERIAL_DONE = 0x0F,

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
    SERIAL_SET_OFFSET = 0x80,
    SERIAL_GET_OFFSET = 0x81,

    SERIAL_RESET_FACTORY_DEFAULTS = 0x60,
    SERIAL_RESET_KVSTORE = 0x70,
};

typedef struct command_t {
    uint8_t id;
    uint8_t n_bytes;
    std::vector<float> params;
};

static const int SERIAL_BAUDRATE =
    57600; // Not actually used when using USB-serial
static const int SERIAL_READ_BUFFER_SIZE = 2048; // 2 kB
static char serialBuffer[SERIAL_READ_BUFFER_SIZE];

void execute_commands(std::vector<command_t> &commands);
std::vector<command_t> check_serial_input(void);
void bytes2command(command_t &cmd, const char *buffer, int bytes_in_buffer);
void command2bytes(command_t &cmd, uint8_t *buffer);

void printAHRSeuler(void);
void printNothing(void);
void printAccCalib(void);
void printMagCalib(void);
void printGyroCalib(void);
void printAccRaw(void);
void printMagRaw(void);
void printGyroRaw(void);
void print_output(void);

void serial_start(std::vector<float> params);
void serial_done(std::vector<float> params);

void mag_set_calib(std::vector<float> params);
void mag_get_calib(std::vector<float> params);
void acc_set_calib(std::vector<float> params);
void acc_get_calib(std::vector<float> params);
void gyro_set_calib(std::vector<float> params);
void gyro_get_calib(std::vector<float> params);
void yaw_set_offset(std::vector<float> params);
void yaw_get_offset(std::vector<float> params);

void set_print_nothing(std::vector<float> params);
void set_print_ahrs(std::vector<float> params);
void set_print_mag_raw(std::vector<float> params);
void set_print_mag_calib(std::vector<float> params);
void set_print_acc_raw(std::vector<float> params);
void set_print_acc_calib(std::vector<float> params);
void set_print_gyro_raw(std::vector<float> params);
void set_print_gyro_calib(std::vector<float> params);

void kv_store_reset(std::vector<float> params);

#endif
