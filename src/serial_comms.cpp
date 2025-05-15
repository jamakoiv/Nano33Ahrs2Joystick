#include <cstring>
#include <map>

#include "Arduino.h"
#include "Fusion/Fusion.h"
#include "ino_globals.h"
#include "kv_storage.h"
#include "serial_comms.h"
#include "utils.h"

// TODO: Serial input & output use a lot of global variables, hard to refactor
// into separate file.
/*
 ---------------- SERIAL OUTPUT PART ------------------------------
*/

using output_func_ptr_t = void (*)(void);
std::map<uint8_t, output_func_ptr_t> output_functions = {
    {SERIAL_PRINT_AHRS, &printAHRSeuler},
    {SERIAL_PRINT_NOTHING, &printNothing},

    {SERIAL_PRINT_ACC_CALIB, &printAccCalib},
    {SERIAL_PRINT_MAG_CALIB, &printMagCalib},
    {SERIAL_PRINT_GYRO_CALIB, &printGyroCalib},

    {SERIAL_PRINT_ACC_RAW, &printAccRaw},
    {SERIAL_PRINT_MAG_RAW, &printMagRaw},
    {SERIAL_PRINT_GYRO_RAW, &printGyroRaw},
};

void printAHRSeuler(void) {
    const FusionEuler euler =
        FusionQuaternionToEuler(FusionAhrsGetQuaternion(&AHRS));

    std::string str =
        "Roll: " + std::to_string(euler.angle.roll) + ", " +
        "Pitch: " + std::to_string(euler.angle.pitch) + ", " +
        "Yaw: " + std::to_string(euler.angle.yaw) + ", " +
        "corr: " + std::to_string(remap_yaw(euler.angle.yaw, AxisOffset.x)) +
        ", " + "Xoff: " + std::to_string(AxisOffset.x) + ", " +
        "Compass: " + std::to_string(CompassHeading);
    Serial.println(str.c_str());
}

void printNothing(void) { return; }

void printAccCalib(void) { Serial.println(CurrentAcc.to_string().c_str()); }
void printMagCalib(void) { Serial.println(CurrentMag.to_string().c_str()); }
void printGyroCalib(void) { Serial.println(CurrentGyro.to_string().c_str()); }

void printAccRaw(void) { Serial.println(rawAcc.to_string().c_str()); }
void printMagRaw(void) { Serial.println(rawMag.to_string().c_str()); }
void printGyroRaw(void) { Serial.println(rawGyro.to_string().c_str()); }

void print_output(void) {
    auto it = output_functions.find(SerialOutputMode);
    if (it == output_functions.end()) {
        Serial.println("Output command not found.");
    } else {
        (it->second)();
    }
    // output_functions[SerialOutputMode]();
}
/*
-------------------- END OF SERIAL OUTPUT PART ------------------
*/

// Updates deltaTime variable.
/*
-------------------- SERIAL INPUT PART --------------------
*/

using input_func_ptr_t = void (*)(std::vector<float> params);
std::map<uint8_t, input_func_ptr_t> input_functions = {
    {SERIAL_START, &serial_start},
    {SERIAL_DONE, &serial_done},

    {SERIAL_MAG_SET_CALIB, &mag_set_calib},
    {SERIAL_MAG_GET_CALIB, &mag_get_calib},

    {SERIAL_ACC_SET_CALIB, &acc_set_calib},
    {SERIAL_ACC_GET_CALIB, &acc_get_calib},

    {SERIAL_GYRO_SET_CALIB, &gyro_set_calib},
    {SERIAL_GYRO_GET_CALIB, &gyro_get_calib},

    {SERIAL_PRINT_NOTHING, &set_print_nothing},
    {SERIAL_PRINT_AHRS, &set_print_ahrs},
    {SERIAL_PRINT_MAG_RAW, &set_print_mag_raw},
    {SERIAL_PRINT_MAG_CALIB, &set_print_mag_calib},
    {SERIAL_PRINT_ACC_RAW, &set_print_acc_raw},
    {SERIAL_PRINT_ACC_CALIB, &set_print_acc_calib},
    {SERIAL_PRINT_GYRO_RAW, &set_print_gyro_raw},
    {SERIAL_PRINT_GYRO_CALIB, &set_print_gyro_calib},

    {SERIAL_RESET_KVSTORE, &kv_store_reset}};

std::vector<command_t> check_serial_input(void) {
    /*
      Check for commands in serial.
    */
    static const std::string COMMAND_DELIMITER = ";";
    static const std::string PARAMETER_DELIMITER = ",";

    std::strncpy(serialBuffer, NULL, SERIAL_READ_BUFFER_SIZE);
    std::vector<command_t> commands;

    while (int bytes_read = Serial.readBytesUntil(';', serialBuffer,
                                                  SERIAL_READ_BUFFER_SIZE)) {
        command_t cmd;
        parse_command(cmd, serialBuffer, bytes_read);
        commands.push_back(cmd);

        // empty the buffer.
        std::strncpy(serialBuffer, NULL, SERIAL_READ_BUFFER_SIZE);
    }

    return commands;
}

void execute_commands(std::vector<command_t> &commands) {
    for (command_t cmd : commands) {

        auto it = input_functions.find(cmd.id);
        if (it == input_functions.end()) {
            Serial.print("Command id not found: 0x");
            Serial.println(cmd.id, HEX);
        } else {
            it->second(cmd.params);
        }
    }
}

void parse_command(command_t &cmd, char *msg, int bytes_in_buffer) {
    /*
     * Parse command send as array of bytes into command_t struct.
     * Sender side should pack the data in python struct format "<BBfff...".
     */
    uint8_t n_header_bytes = 2;
    uint8_t n_data_bytes;

    char *m = const_cast<char *>(msg);
    uint8_t *p = reinterpret_cast<uint8_t *>(m);
    cmd.id = *p;
    p++;
    cmd.n_bytes = *p;
    p++;

    if (cmd.n_bytes != bytes_in_buffer) {
        Serial.println(
            "Warning: Number of bytes read by 'Serial.readBytesUntil' does not "
            "match the number of bytes specified by the message.");
    }

    n_data_bytes = cmd.n_bytes - n_header_bytes;
    float *f = reinterpret_cast<float *>(p);
    for (int i = 0; i < n_data_bytes / sizeof(float); i++) {
        cmd.params.push_back(*f);
        f++;
    }
}

void serial_start(std::vector<float> params) {
    std::string s = int_to_hex(SERIAL_START) + ", Serial start.";
    Serial.println(s.c_str());
}
void serial_done(std::vector<float> params) {
    std::string s = int_to_hex(SERIAL_DONE) + ", Serial done.";
    Serial.println(s.c_str());
}

void mag_set_calib(std::vector<float> params) {
    set_calib_helper(params, MagOffset, MagGain);
    kv_store_save_calibration("MagOffset", MagOffset);
    kv_store_save_calibration("MagGain", MagGain);

    std::string s = int_to_hex(SERIAL_MAG_SET_CALIB) + ", Calibration set.";
    Serial.println(s.c_str());
}

void mag_get_calib(std::vector<float> params) {
    SerialOutputMode = SERIAL_PRINT_NOTHING;
    std::string s = int_to_hex(SERIAL_MAG_SET_CALIB) +
                    ", Magnetic offset (x,y,z): " + MagOffset.to_string() +
                    ", Magnetic gain (x,y,z): " + MagGain.to_string();
    Serial.println(s.c_str());
}

void acc_set_calib(std::vector<float> params) {
    set_calib_helper(params, AccOffset, AccGain);
    kv_store_save_calibration("AccOffset", AccOffset);
    kv_store_save_calibration("AccGain", AccGain);

    std::string s = int_to_hex(SERIAL_ACC_SET_CALIB) + ", Calibration set.";
    Serial.println(s.c_str());
}

void acc_get_calib(std::vector<float> params) {
    SerialOutputMode = SERIAL_PRINT_NOTHING;
    std::string s = int_to_hex(SERIAL_MAG_GET_CALIB) +
                    ", Accelerometer offset (x,y,z): " + AccOffset.to_string();
    +", Accelerometer gain (x,y,z): " + AccGain.to_string();
    Serial.println(s.c_str());
}

void gyro_set_calib(std::vector<float> params) {
    set_calib_helper(params, GyroOffset, GyroGain);
    kv_store_save_calibration("GyroOffset", GyroOffset);
    kv_store_save_calibration("GyroGain", GyroGain);

    std::string s = int_to_hex(SERIAL_GYRO_SET_CALIB) + ", Calibration set.";
    Serial.println(s.c_str());
}

void gyro_get_calib(std::vector<float> params) {
    SerialOutputMode = SERIAL_PRINT_NOTHING;
    std::string s = int_to_hex(SERIAL_GYRO_GET_CALIB) +
                    ", Gyroscope offset (x,y,z): " + GyroOffset.to_string();
    +", Gyroscope gain (x,y,z): " + GyroGain.to_string();
    Serial.println(s.c_str());
}

void yaw_set_offset(std::vector<float> params) {
    set_calib_helper(params, AxisOffset);
    kv_store_save_calibration("AxisOffset", AxisOffset);

    Serial.println("Yaw offset set.");
}

void yaw_get_offset(std::vector<float> params) {
    SerialOutputMode = SERIAL_PRINT_NOTHING;
    std::string str = "Axis Offset (yaw,pitch,roll): " + AxisOffset.to_string();
    Serial.println(str.c_str());
}

/* functions for settings print output mode. */
void set_print_nothing(std::vector<float> params) {
    SerialOutputMode = SERIAL_PRINT_NOTHING;
    Serial.println("Output-mode set to NOTHING.");
}
void set_print_ahrs(std::vector<float> params) {
    SerialOutputMode = SERIAL_PRINT_AHRS;
    Serial.println("Output-mode set to AHRS.");
}
void set_print_mag_raw(std::vector<float> params) {
    SerialOutputMode = SERIAL_PRINT_MAG_RAW;
    Serial.println("Output-mode set to MAGNETOMETER-RAW.");
}
void set_print_mag_calib(std::vector<float> params) {
    SerialOutputMode = SERIAL_PRINT_MAG_CALIB;
    Serial.println("Output-mode set to MAGNETOMETER-CALIBRATED.");
}
void set_print_acc_raw(std::vector<float> params) {
    SerialOutputMode = SERIAL_PRINT_ACC_RAW;
    Serial.println("Output-mode set to ACCELEROMETER-RAW.");
}
void set_print_acc_calib(std::vector<float> params) {
    SerialOutputMode = SERIAL_PRINT_ACC_CALIB;
    Serial.println("Output-mode set to ACCELEROMETER-CALIBRATED.");
}
void set_print_gyro_raw(std::vector<float> params) {
    SerialOutputMode = SERIAL_PRINT_GYRO_RAW;
    Serial.println("Output-mode set to GYROSCOPE-RAW.");
}
void set_print_gyro_calib(std::vector<float> params) {
    SerialOutputMode = SERIAL_PRINT_GYRO_CALIB;
    Serial.println("Output-mode set to GYROSCOPE-RAW.");
}
/*
-------------------- END OF SERIAL INPUT PART --------------------
*/
