#include "serial_comms.h"
#include "Fusion/Fusion.h"
#include "Fusion/FusionMath.h"
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

// TODO: Maybe replace individual MAG_GET, ACC_GET... MAG_SET, ACC_SET commands
// with generic CALIBRATION_GET & CALIBRATION_GET -functions which take target
// as parameter.
void execute_command(command_t &cmd) {
    switch (cmd.id) {
    case SERIAL_SET_PRINT_MODE:
        set_print_mode(cmd.params);
        break;
    case SERIAL_MAG_SET_CALIB:
        mag_set_calib(cmd.params);
        break;
    case SERIAL_MAG_GET_CALIB:
        mag_get_calib();
        break;
    case SERIAL_ACC_SET_CALIB:
        acc_set_calib(cmd.params);
        break;
    case SERIAL_ACC_GET_CALIB:
        acc_get_calib();
        break;
    case SERIAL_GYRO_SET_CALIB:
        gyro_set_calib(cmd.params);
        break;
    case SERIAL_GYRO_GET_CALIB:
        gyro_get_calib();
        break;
    case SERIAL_SET_OFFSET:
        yaw_set_offset(cmd.params);
        break;
    case SERIAL_GET_OFFSET:
        yaw_get_offset();
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

void set_calibration_inertial(const vector<float> &params,
                              FusionMatrix &misalignment,
                              FusionVector &sensitivity, FusionVector &offset) {
    if (params.size() < 9 + 3 + 3) {
        Serial.println("Not enough parameters supplied.");
        return;
    }

    misalignment.element.xx = params[0];
    misalignment.element.xy = params[1];
    misalignment.element.xz = params[2];
    misalignment.element.yx = params[3];
    misalignment.element.yy = params[4];
    misalignment.element.yz = params[5];
    misalignment.element.zx = params[6];
    misalignment.element.zy = params[7];
    misalignment.element.zz = params[8];

    sensitivity.axis.x = params[9];
    sensitivity.axis.y = params[10];
    sensitivity.axis.z = params[11];

    offset.axis.x = params[12];
    offset.axis.y = params[13];
    offset.axis.z = params[14];
}

void set_calibration_magnetic(const vector<float> &params,
                              FusionMatrix &soft_iron_matrix,
                              FusionVector &hard_iron_offset) {
    if (params.size() < 9 + 3) {
        Serial.println("Not enough parameters supplied.");
        return;
    }

    soft_iron_matrix.element.xx = params[0];
    soft_iron_matrix.element.xy = params[1];
    soft_iron_matrix.element.xz = params[2];
    soft_iron_matrix.element.yx = params[3];
    soft_iron_matrix.element.yy = params[4];
    soft_iron_matrix.element.yz = params[5];
    soft_iron_matrix.element.zx = params[6];
    soft_iron_matrix.element.zy = params[7];
    soft_iron_matrix.element.zz = params[8];

    hard_iron_offset.axis.x = params[9];
    hard_iron_offset.axis.y = params[10];
    hard_iron_offset.axis.z = params[11];
}

void get_calibration_inertial(command_t &cmd, const FusionMatrix &misalignment,
                              const FusionVector &sensitivity,
                              const FusionVector &offset) {
    cmd.n_params = 9 + 3 + 3;

    cmd.params.push_back(misalignment.element.xx);
    cmd.params.push_back(misalignment.element.xy);
    cmd.params.push_back(misalignment.element.xz);
    cmd.params.push_back(misalignment.element.yx);
    cmd.params.push_back(misalignment.element.yy);
    cmd.params.push_back(misalignment.element.yz);
    cmd.params.push_back(misalignment.element.zx);
    cmd.params.push_back(misalignment.element.zy);
    cmd.params.push_back(misalignment.element.zz);

    cmd.params.push_back(sensitivity.axis.x);
    cmd.params.push_back(sensitivity.axis.y);
    cmd.params.push_back(sensitivity.axis.z);

    cmd.params.push_back(offset.axis.x);
    cmd.params.push_back(offset.axis.y);
    cmd.params.push_back(offset.axis.z);
}

void get_calibration_magnetic(command_t &cmd,
                              const FusionMatrix &soft_iron_matrix,
                              const FusionVector &hard_iron_offset) {
    cmd.n_params = 9 + 3;

    cmd.params.push_back(soft_iron_matrix.element.xx);
    cmd.params.push_back(soft_iron_matrix.element.xy);
    cmd.params.push_back(soft_iron_matrix.element.xz);
    cmd.params.push_back(soft_iron_matrix.element.yx);
    cmd.params.push_back(soft_iron_matrix.element.yy);
    cmd.params.push_back(soft_iron_matrix.element.yz);
    cmd.params.push_back(soft_iron_matrix.element.zx);
    cmd.params.push_back(soft_iron_matrix.element.zy);
    cmd.params.push_back(soft_iron_matrix.element.zz);

    cmd.params.push_back(hard_iron_offset.axis.x);
    cmd.params.push_back(hard_iron_offset.axis.y);
    cmd.params.push_back(hard_iron_offset.axis.z);
}

void mag_set_calib(vector<float> params) {
    SerialOutputMode = SERIAL_PRINT_NOTHING;

    set_calibration_magnetic(params, soft_iron, hard_iron);
    kv_store_save_calibration("MagOffset", hard_iron);
    kv_store_save_calibration("MagGain", soft_iron);

    string s = int_to_hex(SERIAL_MAG_SET_CALIB) + ", Calibration set;";
    Serial.println(s.c_str());
}

void mag_get_calib(void) {
    SerialOutputMode = SERIAL_PRINT_NOTHING;

    command_t cmd;
    cmd.id = SERIAL_MAG_GET_CALIB;
    get_calibration_magnetic(cmd, soft_iron, hard_iron);

    string msg = create_message(cmd);
    Serial.println(msg.c_str());
}

void acc_set_calib(vector<float> params) {
    SerialOutputMode = SERIAL_PRINT_NOTHING;

    set_calibration_inertial(params, acc_misalignment, acc_gain, acc_offset);
    kv_store_save_calibration("AccOffset", acc_offset);
    kv_store_save_calibration("AccGain", acc_gain);

    string s = int_to_hex(SERIAL_ACC_SET_CALIB) + ", Calibration set;";
    Serial.println(s.c_str());
}

void acc_get_calib(void) {
    SerialOutputMode = SERIAL_PRINT_NOTHING;

    command_t cmd;
    cmd.id = SERIAL_ACC_GET_CALIB;
    get_calibration_inertial(cmd, acc_misalignment, acc_gain, acc_offset);

    string msg = create_message(cmd);
    Serial.println(msg.c_str());
}

void gyro_set_calib(vector<float> params) {
    SerialOutputMode = SERIAL_PRINT_NOTHING;

    set_calibration_inertial(params, gyro_misalignment, gyro_gain, gyro_offset);
    kv_store_save_calibration("GyroOffset", gyro_offset);
    kv_store_save_calibration("GyroGain", gyro_gain);

    string s = int_to_hex(SERIAL_GYRO_SET_CALIB) + ", Calibration set;";
    Serial.println(s.c_str());
}

void gyro_get_calib(void) {
    SerialOutputMode = SERIAL_PRINT_NOTHING;

    command_t cmd;
    cmd.id = SERIAL_GYRO_GET_CALIB;
    get_calibration_inertial(cmd, gyro_misalignment, gyro_gain, gyro_offset);

    string msg = create_message(cmd);
    Serial.println(msg.c_str());
}

void yaw_set_offset(vector<float> params) {
    SerialOutputMode = SERIAL_PRINT_NOTHING;

    AxisOffset.axis.x = params[0];
    AxisOffset.axis.x = params[1];
    AxisOffset.axis.x = params[2];

    kv_store_save_calibration("AxisOffset", AxisOffset);
    Serial.println("Yaw offset set;");
}
void yaw_get_offset(void) {
    SerialOutputMode = SERIAL_PRINT_NOTHING;

    command_t cmd;
    cmd.id = SERIAL_GET_OFFSET;
    cmd.n_params = 3;
    cmd.params.push_back(AxisOffset.axis.x);
    cmd.params.push_back(AxisOffset.axis.y);
    cmd.params.push_back(AxisOffset.axis.z);

    string msg = create_message(cmd);
    Serial.println(msg.c_str());
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
