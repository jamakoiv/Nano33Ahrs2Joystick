#include <cstring>
#include <map>

#include "Fusion/Fusion.h"
#include "ino_globals.h"
#include "kv_storage.h"
#include "serial_comms.h"
#include "serial_utils.h"
#include "utils.h"
#include <Arduino.h>

// TODO: Serial input & output use a lot of global variables, hard to refactor
// into separate file.
/*
 ---------------- SERIAL OUTPUT PART ------------------------------
*/

void printAHRSeuler(void) {
    const FusionEuler euler =
        FusionQuaternionToEuler(FusionAhrsGetQuaternion(&AHRS));

    std::string str =
        "Roll: " + std::to_string(euler.angle.roll) + ", " +
        "Pitch: " + std::to_string(euler.angle.pitch) + ", " +
        "Yaw: " + std::to_string(euler.angle.yaw) + ", " + "corr: " +
        std::to_string(remap_yaw(euler.angle.yaw, AxisOffset.axis.x)) + ", " +
        "Xoff: " + std::to_string(AxisOffset.axis.x) + ", " +
        "Compass: " + std::to_string(CompassHeading);
    Serial.println(str.c_str());
}

void printNothing(void) { return; }

void printAccCalib(void) {
    char buf[1024];
    sprintf(buf, "%f, %f, %f", acc_calibrated.axis.x, acc_calibrated.axis.y,
            acc_calibrated.axis.z);
    Serial.println(buf);
}

void printMagCalib(void) {
    char buf[1024];
    sprintf(buf, "%f, %f, %f", mag_calibrated.axis.x, mag_calibrated.axis.y,
            mag_calibrated.axis.z);
    Serial.println(buf);
}

void printGyroCalib(void) {
    char buf[1024];
    sprintf(buf, "%f, %f, %f", gyro_calibrated.axis.x, gyro_calibrated.axis.y,
            gyro_calibrated.axis.z);
    Serial.println(buf);
}

void printAccRaw(void) {
    char buf[1024];
    sprintf(buf, "%f, %f, %f", acc_raw.axis.x, acc_raw.axis.y, acc_raw.axis.z);
    Serial.println(buf);
}

void printMagRaw(void) {
    char buf[1024];
    sprintf(buf, "%f, %f, %f", mag_raw.axis.x, mag_raw.axis.y, mag_raw.axis.z);
    Serial.println(buf);
}

void printGyroRaw(void) {
    char buf[1024];
    sprintf(buf, "%f, %f, %f", gyro_raw.axis.x, gyro_raw.axis.y,
            gyro_raw.axis.z);
    Serial.println(buf);
}

// INFO: Better than the old 'get pointer to function from map and call that'...
void print_output(void) {
    switch (SerialOutputMode) {
    case SERIAL_PRINT_NOTHING:
        printNothing();
        break;
    case SERIAL_PRINT_AHRS:
        printAHRSeuler();
        break;
    case SERIAL_PRINT_AHRS_DEBUG:
        // printAHRSeulerDebug();
        break;
    case SERIAL_PRINT_ACC_CALIB:
        printAccCalib();
        break;
    case SERIAL_PRINT_MAG_CALIB:
        printMagCalib();
        break;
    case SERIAL_PRINT_GYRO_CALIB:
        printGyroCalib();
        break;
    case SERIAL_PRINT_ACC_RAW:
        printAccRaw();
        break;
    case SERIAL_PRINT_MAG_RAW:
        printMagRaw();
        break;
    case SERIAL_PRINT_GYRO_RAW:
        printGyroRaw();
        break;
    default:
        Serial.println("Error: Output mode not found;");
    };
}
/*
-------------------- END OF SERIAL OUTPUT PART ------------------
*/

/*
-------------------- SERIAL INPUT PART --------------------
*/

// TODO: We cannot replace Serial.read from this. Need to move it to somewhere
// where we can better contain all Arduino-specific code to one place.
std::vector<command_t> check_serial_input(void) {
    /*
      Check for commands in serial.
    */
    static const std::string COMMAND_DELIMITER = ";";
    static const std::string PARAMETER_DELIMITER = ",";

    std::strncpy(serialBuffer, NULL, SERIAL_READ_BUFFER_SIZE);
    std::vector<command_t> commands;

    // BUG: If there ever happens to be a number for which the
    // byte-representation contains ';', like 46.75099945 which gives
    // b'\x06\x01;B' the message is read incomplete.
    // TODO: Change comms to use ASCII SOH <header_bytes> STX <data_bytes> ETX
    // EOT instead of this self-made fragile system.
    while (int bytes_read = Serial.readBytesUntil(';', serialBuffer,
                                                  SERIAL_READ_BUFFER_SIZE)) {
        command_t cmd;
        bytes2command(cmd, serialBuffer, bytes_read);
        commands.push_back(cmd);

        // empty the buffer.
        std::strncpy(serialBuffer, NULL, SERIAL_READ_BUFFER_SIZE);
    }

    return commands;
}

void execute_commands(std::vector<command_t> &commands) {
    for (command_t cmd : commands) {
        switch (cmd.id) {
        case SERIAL_SET_PRINT_MODE:
            set_print_mode(cmd.params);
            break;
        case SERIAL_MAG_SET_CALIB:
            mag_set_calib(cmd.params);
            break;
        case SERIAL_MAG_GET_CALIB:
            mag_get_calib(cmd.params);
            break;
        case SERIAL_ACC_SET_CALIB:
            acc_set_calib(cmd.params);
            break;
        case SERIAL_ACC_GET_CALIB:
            acc_get_calib(cmd.params);
            break;
        case SERIAL_GYRO_SET_CALIB:
            gyro_set_calib(cmd.params);
            break;
        case SERIAL_GYRO_GET_CALIB:
            gyro_get_calib(cmd.params);
            break;
        case SERIAL_SET_OFFSET:
            yaw_set_offset(cmd.params);
            break;
        case SERIAL_GET_OFFSET:
            yaw_get_offset(cmd.params);
            break;
        case SERIAL_RESET_KVSTORE:
            kv_store_reset();
            break;

        default:
            break;
        }
    }
}

void bytes2command(command_t &cmd, const char *msg, int bytes_in_buffer) {
    /*
     * Parse command send as array of bytes into command_t struct.
     * Sender side should pack the data in python struct format "<BBfff...".
     */

    // NOTE: msg is defined as const char so you can also feed parameter
    // from std::string.c_str().
    uint8_t n_header_bytes = 2;
    uint8_t n_data_bytes;

    uint8_t *p = reinterpret_cast<uint8_t *>(const_cast<char *>(msg));
    cmd.id = *p;
    p++;
    cmd.n_bytes = *p;
    p++;

    if (cmd.n_bytes != bytes_in_buffer) {
        std::string err =
            "Warning: Number of bytes read by 'Serial.readBytesUntil' " +
            std::to_string(cmd.n_bytes) +
            " does not match the number of bytes specified by the message " +
            std::to_string(bytes_in_buffer) + ";";
        Serial.println(err.c_str());
    }

    n_data_bytes = cmd.n_bytes - n_header_bytes;
    float *f = reinterpret_cast<float *>(p);
    for (int i = 0; i < n_data_bytes / sizeof(float); i++) {
        cmd.params.push_back(*f);
        f++;
    }
}

void command2bytes(command_t &cmd, uint8_t *buffer) {
    /*
     * Parse command into array of bytes for sending over serial.
     */
    uint8_t stop_byte = ';';
    uint8_t *p = buffer;

    *p = cmd.id;
    p++;
    *p = cmd.n_bytes;
    p++;

    float *f = reinterpret_cast<float *>(p);
    for (float param : cmd.params) {
        *f = param;
        f++;
    }

    p = reinterpret_cast<uint8_t *>(f);
    *p = stop_byte;
}

void mag_set_calib(std::vector<float> params) {
    SerialOutputMode = SERIAL_PRINT_NOTHING;

    set_calib_helper(params, hard_iron, soft_iron);
    kv_store_save_calibration("MagOffset", hard_iron);
    kv_store_save_calibration("MagGain", soft_iron);

    std::string s = int_to_hex(SERIAL_MAG_SET_CALIB) + ", Calibration set;";
    Serial.println(s.c_str());
}

void mag_get_calib(std::vector<float> params) {
    SerialOutputMode = SERIAL_PRINT_NOTHING;

    uint8_t buffer[1024];
    command_t cmd;
    cmd.id = SERIAL_MAG_GET_CALIB;
    cmd.n_bytes = 2 + 6 * sizeof(float);
    cmd.params.push_back(hard_iron.axis.x);
    cmd.params.push_back(hard_iron.axis.y);
    cmd.params.push_back(hard_iron.axis.z);
    cmd.params.push_back(1.0 / soft_iron.element.xx);
    cmd.params.push_back(1.0 / soft_iron.element.yy);
    cmd.params.push_back(1.0 / soft_iron.element.zz);

    // Serial.println(soft_iron.element.xx, 5);
    // Serial.println(soft_iron.element.yy, 5);
    // Serial.println(soft_iron.element.zz, 5);

    command2bytes(cmd, buffer);
    Serial.write(buffer, cmd.n_bytes + 1);
    Serial.println("");
}

void acc_set_calib(std::vector<float> params) {
    set_calib_helper(params, acc_offset, acc_gain);
    kv_store_save_calibration("AccOffset", acc_offset);
    kv_store_save_calibration("AccGain", acc_gain);

    std::string s = int_to_hex(SERIAL_ACC_SET_CALIB) + ", Calibration set;";
    Serial.println(s.c_str());
}

void acc_get_calib(std::vector<float> params) {
    SerialOutputMode = SERIAL_PRINT_NOTHING;

    uint8_t buffer[1024];
    command_t cmd;
    cmd.id = SERIAL_ACC_GET_CALIB;
    cmd.n_bytes = 2 + 6 * sizeof(float);
    cmd.params.push_back(acc_offset.axis.x);
    cmd.params.push_back(acc_offset.axis.y);
    cmd.params.push_back(acc_offset.axis.z);
    cmd.params.push_back(acc_gain.axis.x);
    cmd.params.push_back(acc_gain.axis.y);
    cmd.params.push_back(acc_gain.axis.z);

    command2bytes(cmd, buffer);
    Serial.write(buffer, cmd.n_bytes + 1);
    Serial.println("");
}

void gyro_set_calib(std::vector<float> params) {
    set_calib_helper(params, gyro_offset, gyro_gain);
    kv_store_save_calibration("GyroOffset", gyro_offset);
    kv_store_save_calibration("GyroGain", gyro_gain);

    std::string s = int_to_hex(SERIAL_GYRO_SET_CALIB) + ", Calibration set;";
    Serial.println(s.c_str());
}

void gyro_get_calib(std::vector<float> params) {
    SerialOutputMode = SERIAL_PRINT_NOTHING;

    uint8_t buffer[1024];
    command_t cmd;
    cmd.id = SERIAL_GYRO_GET_CALIB;
    cmd.n_bytes = 2 + 6 * sizeof(float);
    cmd.params.push_back(gyro_offset.axis.x);
    cmd.params.push_back(gyro_offset.axis.y);
    cmd.params.push_back(gyro_offset.axis.z);
    cmd.params.push_back(gyro_gain.axis.x);
    cmd.params.push_back(gyro_gain.axis.y);
    cmd.params.push_back(gyro_gain.axis.z);

    command2bytes(cmd, buffer);
    Serial.write(buffer, cmd.n_bytes + 1);
    Serial.println("");
}

void yaw_set_offset(std::vector<float> params) {
    set_calib_helper(params, AxisOffset);
    kv_store_save_calibration("AxisOffset", AxisOffset);

    Serial.println("Yaw offset set;");
}
void yaw_get_offset(std::vector<float> params) {
    SerialOutputMode = SERIAL_PRINT_NOTHING;
    std::string str =
        "Axis Offset (yaw,pitch,roll): " + std::to_string(AxisOffset.axis.x) +
        ", " + std::to_string(AxisOffset.axis.y) + ", " +
        std::to_string(AxisOffset.axis.z) + ";";
    Serial.println(str.c_str());
}

void set_print_mode(std::vector<float> params) {
    if (params.size() < 1) {
        Serial.println("Error: No parameters given;");
        return;
    }

    SerialOutputMode = static_cast<uint8_t>(params[0]);
    std::string s = "Output mode set to " + int_to_hex(SerialOutputMode) + ";";
    Serial.println(s.c_str());
}

/*
-------------------- END OF SERIAL INPUT PART --------------------
*/
