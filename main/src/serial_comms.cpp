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

// TODO: Serial input & output use a lot of global variables, hard to refactor
// into separate file.

/*
 ---------------- SERIAL OUTPUT PART ------------------------------
*/

string printAHRSeuler(void) {
    const FusionEuler euler =
        FusionQuaternionToEuler(FusionAhrsGetQuaternion(&AHRS));

    string msg = "Roll: " + std::to_string(euler.angle.roll) + ", " +
                 "Pitch: " + std::to_string(euler.angle.pitch) + ", " +
                 "Yaw: " + std::to_string(euler.angle.yaw) + ", " +
                 "Compass: " + std::to_string(CompassHeading) + ", ";
    return msg;
}

string printAHRSeulerDebug(void) {
    const FusionEuler euler =
        FusionQuaternionToEuler(FusionAhrsGetQuaternion(&AHRS));

    string msg =
        "Roll: " + std::to_string(euler.angle.roll) + ", " +
        "Pitch: " + std::to_string(euler.angle.pitch) + ", " +
        "Yaw: " + std::to_string(euler.angle.yaw) + ", " +
        "Compass: " + std::to_string(CompassHeading) + ", " +
        "Init: " + std::to_string(AHRS.initialising) + ", " +
        "RateRecovery: " + std::to_string(AHRS.angularRateRecovery) + ", " +
        "AccIgnored: " + std::to_string(AHRS.accelerometerIgnored) + ", " +
        "MagIgnored: " + std::to_string(AHRS.magnetometerIgnored) + ", ";
    return msg;
}

string printFusionVector(FusionVector vec) {
    string str = std::to_string(vec.axis.x) + ", " +
                 std::to_string(vec.axis.y) + ", " +
                 std::to_string(vec.axis.z) + ", ";
    return str;
}

string printMagGyroRaw(void) {
    static const float MICROSECONDS_TO_SECONDS = 1.0f / 1000000.0f;

    float time = static_cast<float>(IMU_timeStamp) * MICROSECONDS_TO_SECONDS;
    string str = std::to_string(time) + ", " + printFusionVector(mag_raw) +
                 printFusionVector(gyro_raw);
    return str;
}

string printFusionMatrix(FusionMatrix mat) {
    // clang-format off
    string str = std::to_string(mat.element.xx) + ", " +
                 std::to_string(mat.element.xy) + ", " +
                 std::to_string(mat.element.xz) + "\n" +

                 std::to_string(mat.element.yx) + ", " +
                 std::to_string(mat.element.yy) + ", " +
                 std::to_string(mat.element.yz) + "\n" +

                 std::to_string(mat.element.zx) + ", " +
                 std::to_string(mat.element.zy) + ", " +
                 std::to_string(mat.element.zz) + "\n";
    // clang-format on
    return str;
};

// INFO: Better than the old 'get pointer to function from map and call that'...
string print_output(void) {
    switch (SerialOutputMode) {
    case SERIAL_PRINT_NOTHING:
        return string();
    case SERIAL_PRINT_AHRS:
        return printAHRSeuler();
    case SERIAL_PRINT_AHRS_DEBUG:
        return printAHRSeulerDebug();
    case SERIAL_PRINT_ACC_CALIB:
        return printFusionVector(acc_calibrated);
    case SERIAL_PRINT_MAG_CALIB:
        return printFusionVector(mag_calibrated);
    case SERIAL_PRINT_GYRO_CALIB:
        return printFusionVector(gyro_calibrated);
    case SERIAL_PRINT_ACC_RAW:
        return printFusionVector(acc_raw);
    case SERIAL_PRINT_MAG_RAW:
        return printFusionVector(mag_raw);
    case SERIAL_PRINT_GYRO_RAW:
        return printFusionVector(gyro_raw);
    case SERIAL_PRINT_MAG_GYRO_RAW:
        return printMagGyroRaw();
    default:
        string msg = "Error: Output mode " + std::to_string(SerialOutputMode) +
                     " is not valid.";
        return msg;
    };
}
/*
-------------------- END OF SERIAL OUTPUT PART ------------------
*/

/*
-------------------- SERIAL INPUT PART --------------------------
*/

std::string ahrs_set_settings(std::vector<float> params);
std::string ahrs_get_settings(void);

// TODO: Maybe replace individual MAG_GET, ACC_GET... MAG_SET, ACC_SET commands
// with generic CALIBRATION_GET & CALIBRATION_GET -functions which take target
// as parameter.
std::string execute_command(command_t &cmd) {
    switch (cmd.id) {
    case SERIAL_SET_PRINT_MODE:
        return set_print_mode(cmd.params);
    case SERIAL_MAG_SET_CALIB:
        return mag_set_calib(cmd.params);
    case SERIAL_MAG_GET_CALIB:
        return mag_get_calib();
    case SERIAL_ACC_SET_CALIB:
        return acc_set_calib(cmd.params);
    case SERIAL_ACC_GET_CALIB:
        return acc_get_calib();
    case SERIAL_GYRO_SET_CALIB:
        return gyro_set_calib(cmd.params);
    case SERIAL_GYRO_GET_CALIB:
        return gyro_get_calib();
    case SERIAL_SET_OFFSET:
        return yaw_set_offset(cmd.params);
    case SERIAL_GET_OFFSET:
        return yaw_get_offset();
    case SERIAL_RESET_KVSTORE:
        return _kv_store_reset();
    case SERIAL_AHRS_SET_SETTINGS:
        return ahrs_set_settings(cmd.params);
    case SERIAL_AHRS_GET_SETTINGS:
        return ahrs_get_settings();
    default:
        string msg = "Command " + std::to_string(cmd.id) + " not found.";
        return msg;
    }
}

void set_calibration_inertial(const vector<float> &params,
                              FusionMatrix &misalignment,
                              FusionVector &sensitivity, FusionVector &offset) {
    if (params.size() < 9 + 3 + 3) {
        Serial.println(
            "Not enough parameters supplied."); // TODO: Still printing in this
                                                // file...
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
        Serial.println(
            "Not enough parameters supplied."); // TODO: Still printing in this
                                                // file...
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

// TODO: Lots of repeating of essentially the same function.
//
string mag_set_calib(vector<float> params) {
    SerialOutputMode = SERIAL_PRINT_NOTHING;

    set_calibration_magnetic(params, soft_iron, hard_iron);
    kv_store_save_calibration("MagOffset", hard_iron);
    kv_store_save_calibration("MagGain", soft_iron);

    string s = int_to_hex(SERIAL_MAG_SET_CALIB) + ", Calibration set;";
    return s;
}

string mag_get_calib(void) {
    SerialOutputMode = SERIAL_PRINT_NOTHING;

    command_t cmd;
    cmd.id = SERIAL_MAG_GET_CALIB;
    get_calibration_magnetic(cmd, soft_iron, hard_iron);

    string msg = create_message(cmd);
    return msg;
}

string acc_set_calib(vector<float> params) {
    SerialOutputMode = SERIAL_PRINT_NOTHING;

    set_calibration_inertial(params, acc_misalignment, acc_gain, acc_offset);
    kv_store_save_calibration("AccOffset", acc_offset);
    kv_store_save_calibration("AccGain", acc_gain);

    string s = int_to_hex(SERIAL_ACC_SET_CALIB) + ", Calibration set;";
    return s;
}

string acc_get_calib(void) {
    SerialOutputMode = SERIAL_PRINT_NOTHING;

    command_t cmd;
    cmd.id = SERIAL_ACC_GET_CALIB;
    get_calibration_inertial(cmd, acc_misalignment, acc_gain, acc_offset);

    string msg = create_message(cmd);
    return msg;
}

string gyro_set_calib(vector<float> params) {
    SerialOutputMode = SERIAL_PRINT_NOTHING;

    set_calibration_inertial(params, gyro_misalignment, gyro_gain, gyro_offset);
    kv_store_save_calibration("GyroOffset", gyro_offset);
    kv_store_save_calibration("GyroGain", gyro_gain);

    string s = int_to_hex(SERIAL_GYRO_SET_CALIB) + ", Calibration set;";
    return s;
}

string gyro_get_calib(void) {
    SerialOutputMode = SERIAL_PRINT_NOTHING;

    command_t cmd;
    cmd.id = SERIAL_GYRO_GET_CALIB;
    get_calibration_inertial(cmd, gyro_misalignment, gyro_gain, gyro_offset);

    string msg = create_message(cmd);
    return msg;
}

string yaw_set_offset(vector<float> params) {
    SerialOutputMode = SERIAL_PRINT_NOTHING;

    AxisOffset.axis.x = params[0];
    AxisOffset.axis.x = params[1];
    AxisOffset.axis.x = params[2];

    kv_store_save_calibration("AxisOffset", AxisOffset);
    string msg("Yaw offset set;");
    return msg;
}
string yaw_get_offset(void) {
    SerialOutputMode = SERIAL_PRINT_NOTHING;

    command_t cmd;
    cmd.id = SERIAL_GET_OFFSET;
    cmd.n_params = 3;
    cmd.params.push_back(AxisOffset.axis.x);
    cmd.params.push_back(AxisOffset.axis.y);
    cmd.params.push_back(AxisOffset.axis.z);

    string msg = create_message(cmd);
    return msg;
}

string set_print_mode(vector<float> params) {
    if (params.size() < 1) {
        string err("Error: No serial output mode given");
        return err;
    }

    SerialOutputMode = static_cast<uint8_t>(params[0]);
    string msg = "Output mode set to " + int_to_hex(SerialOutputMode) + ";";
    return msg;
}

string ahrs_set_settings(vector<float> params) {
    if (params.size() < 4) {
        string err("Error: Not enough parameters given");
        return err;
    }
    AHRSsettings.gain = params[0];
    AHRSsettings.accelerationRejection = params[1];
    AHRSsettings.magneticRejection = params[2];
    AHRSsettings.recoveryTriggerPeriod = params[3];

    FusionAhrsSetSettings(&AHRS, &AHRSsettings);
    FusionAhrsReset(&AHRS);

    string msg = "AHRS settings set;";
    return msg;
}

string ahrs_get_settings(void) {
    command_t cmd;
    cmd.id = SERIAL_AHRS_GET_SETTINGS;
    cmd.n_params = 4;
    cmd.params.push_back(AHRSsettings.gain);
    cmd.params.push_back(AHRSsettings.accelerationRejection);
    cmd.params.push_back(AHRSsettings.magneticRejection);
    cmd.params.push_back(AHRSsettings.recoveryTriggerPeriod);

    string msg = create_message(cmd);
    return msg;
}

// TODO: Stupid wrapper function...
string _kv_store_reset() {
    kv_store_reset();

    string msg("Resetting KV-store");
    return msg;
}

/*
-------------------- END OF SERIAL INPUT PART --------------------
*/
