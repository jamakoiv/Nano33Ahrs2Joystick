#include "serial_comms.h"
#include "Fusion/Fusion.h"
#include "ino_globals.h"
#include "kv_storage.h"
#include "serial_utils.h"
#include "utils.h"

#include <Arduino.h>
#include <cstring>

using std::string;
using std::vector;

// TODO: Do we need to do any printing in this file, or just have every print
// function to return a string and print in the main-file?
//
// TODO: Serial input & output use a lot of global variables, hard to refactor
// into separate file.

/*
 ---------------- SERIAL OUTPUT PART ------------------------------
*/

void printAHRSeuler(void) {
    const FusionEuler euler =
        FusionQuaternionToEuler(FusionAhrsGetQuaternion(&AHRS));

    string str = "Roll: " + std::to_string(euler.angle.roll) + ", " +
                 "Pitch: " + std::to_string(euler.angle.pitch) + ", " +
                 "Yaw: " + std::to_string(euler.angle.yaw) + ", " +
                 "Compass: " + std::to_string(CompassHeading);
    Serial.println(str.c_str());
}

void printAHRSeulerDebug(void) {
    // TODO: implement.
}

void printNothing(void) { return; }

void printFusionVector(FusionVector vec) {
    string str = std::to_string(vec.axis.x) + ", " +
                 std::to_string(vec.axis.y) + ", " + std::to_string(vec.axis.z);
    Serial.println(str.c_str());
}

void printFusionMatrix(FusionMatrix mat) {
    // TODO: implement.
};

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
        printFusionVector(acc_calibrated);
        break;
    case SERIAL_PRINT_MAG_CALIB:
        printFusionVector(mag_calibrated);
        break;
    case SERIAL_PRINT_GYRO_CALIB:
        printFusionVector(gyro_calibrated);
        break;
    case SERIAL_PRINT_ACC_RAW:
        printFusionVector(acc_raw);
        break;
    case SERIAL_PRINT_MAG_RAW:
        printFusionVector(mag_raw);
        break;
    case SERIAL_PRINT_GYRO_RAW:
        printFusionVector(gyro_raw);
        break;
    default:
        Serial.println("Error: Output mode not found;");
    };
}
/*
-------------------- END OF SERIAL OUTPUT PART ------------------
*/

/*
-------------------- SERIAL INPUT PART --------------------------
*/
void execute_commands(command_t &cmd) {
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
        string msg = "Command " + std::to_string(cmd.id) + " not found.";
        Serial.println(msg.c_str());
        break;
    }
}

void mag_set_calib(vector<float> params) {
    SerialOutputMode = SERIAL_PRINT_NOTHING;

    set_calib_helper(params, hard_iron, soft_iron);
    kv_store_save_calibration("MagOffset", hard_iron);
    kv_store_save_calibration("MagGain", soft_iron);

    string s = int_to_hex(SERIAL_MAG_SET_CALIB) + ", Calibration set;";
    Serial.println(s.c_str());
}

void mag_get_calib(vector<float> params) {
    SerialOutputMode = SERIAL_PRINT_NOTHING;

    command_t cmd;
    cmd.id = SERIAL_MAG_GET_CALIB;
    cmd.n_params = 6;
    cmd.params.push_back(hard_iron.axis.x);
    cmd.params.push_back(hard_iron.axis.y);
    cmd.params.push_back(hard_iron.axis.z);
    cmd.params.push_back(1.0 / soft_iron.element.xx);
    cmd.params.push_back(1.0 / soft_iron.element.yy);
    cmd.params.push_back(1.0 / soft_iron.element.zz);

    // Serial.println(soft_iron.element.xx, 5);
    // Serial.println(soft_iron.element.yy, 5);
    // Serial.println(soft_iron.element.zz, 5);

    string msg = create_message(cmd);
    Serial.println(msg.c_str());
}

void acc_set_calib(vector<float> params) {
    set_calib_helper(params, acc_offset, acc_gain);
    kv_store_save_calibration("AccOffset", acc_offset);
    kv_store_save_calibration("AccGain", acc_gain);

    string s = int_to_hex(SERIAL_ACC_SET_CALIB) + ", Calibration set;";
    Serial.println(s.c_str());
}

void acc_get_calib(vector<float> params) {
    SerialOutputMode = SERIAL_PRINT_NOTHING;

    command_t cmd;
    cmd.id = SERIAL_ACC_GET_CALIB;
    cmd.n_params = 6;
    cmd.params.push_back(acc_offset.axis.x);
    cmd.params.push_back(acc_offset.axis.y);
    cmd.params.push_back(acc_offset.axis.z);
    cmd.params.push_back(acc_gain.axis.x);
    cmd.params.push_back(acc_gain.axis.y);
    cmd.params.push_back(acc_gain.axis.z);

    string msg = create_message(cmd);
    Serial.println(msg.c_str());
}

void gyro_set_calib(vector<float> params) {
    set_calib_helper(params, gyro_offset, gyro_gain);
    kv_store_save_calibration("GyroOffset", gyro_offset);
    kv_store_save_calibration("GyroGain", gyro_gain);

    string s = int_to_hex(SERIAL_GYRO_SET_CALIB) + ", Calibration set;";
    Serial.println(s.c_str());
}

void gyro_get_calib(vector<float> params) {
    SerialOutputMode = SERIAL_PRINT_NOTHING;

    command_t cmd;
    cmd.id = SERIAL_GYRO_GET_CALIB;
    cmd.n_params = 6;
    cmd.params.push_back(gyro_offset.axis.x);
    cmd.params.push_back(gyro_offset.axis.y);
    cmd.params.push_back(gyro_offset.axis.z);
    cmd.params.push_back(gyro_gain.axis.x);
    cmd.params.push_back(gyro_gain.axis.y);
    cmd.params.push_back(gyro_gain.axis.z);

    string msg = create_message(cmd);
    Serial.println(msg.c_str());
}

void yaw_set_offset(vector<float> params) {
    set_calib_helper(params, AxisOffset);
    kv_store_save_calibration("AxisOffset", AxisOffset);

    Serial.println("Yaw offset set;");
}
void yaw_get_offset(vector<float> params) {
    SerialOutputMode = SERIAL_PRINT_NOTHING;
    string str =
        "Axis Offset (yaw,pitch,roll): " + std::to_string(AxisOffset.axis.x) +
        ", " + std::to_string(AxisOffset.axis.y) + ", " +
        std::to_string(AxisOffset.axis.z) + ";";
    Serial.println(str.c_str());
}

void set_print_mode(vector<float> params) {
    if (params.size() < 1) {
        Serial.println("Error: No parameters given;");
        return;
    }

    SerialOutputMode = static_cast<uint8_t>(params[0]);
    string msg = "Output mode set to " + int_to_hex(SerialOutputMode) + ";";
    Serial.println(msg.c_str());
}

/*
-------------------- END OF SERIAL INPUT PART --------------------
*/
