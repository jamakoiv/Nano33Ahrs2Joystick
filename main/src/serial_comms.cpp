#include "serial_comms.h"
#include "Fusion/Fusion.h"
#include "Fusion/FusionMath.h"
#include "ino_globals.h"
#include "kv_storage.h"
#include "serial_utils.h"
#include "utils.h"

#include <Arduino.h>
#include <cstring>

// TODO: Serial input & output use a lot of global variables, hard to refactor
// into separate file.

/*
 ---------------- SERIAL OUTPUT PART ------------------------------
*/

std::string printAHRSeuler(void) {
    const FusionEuler euler =
        FusionQuaternionToEuler(FusionAhrsGetQuaternion(&AHRS));

    string msg = "Roll: " + std::to_string(euler.angle.roll) + ", " +
                 "Pitch: " + std::to_string(euler.angle.pitch) + ", " +
                 "Yaw: " + std::to_string(euler.angle.yaw) + ", " +
                 "Compass: " + std::to_string(CompassHeading) + ", ";
    return msg;
}

std::string printAHRSeulerDebug(void) {
    const FusionEuler euler =
        FusionQuaternionToEuler(FusionAhrsGetQuaternion(&AHRS));

    std::string msg =
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

std::string printFusionVector(FusionVector vec) {
    std::string str = std::to_string(vec.axis.x) + ", " +
                      std::to_string(vec.axis.y) + ", " +
                      std::to_string(vec.axis.z) + ", ";
    return str;
}

std::string printMagGyroRaw(void) {
    static const float MICROSECONDS_TO_SECONDS = 1.0f / 1000000.0f;

    float time = static_cast<float>(IMU_timeStamp) * MICROSECONDS_TO_SECONDS;
    std::string str = std::to_string(time) + ", " + printFusionVector(mag_raw) +
                      printFusionVector(gyro_raw);
    return str;
}

std::string printFusionMatrix(FusionMatrix mat) {
    // clang-format off
    std::string str = std::to_string(mat.element.xx) + ", " +
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
std::string print_output(void) {
    switch (SerialOutputMode) {
    case SERIAL_PRINT_NOTHING:
        return std::string();
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

std::string misc_set_settings(std::vector<float> params);
std::string misc_get_settings(void);

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
    case SERIAL_RESET_KVSTORE:
        return _kv_store_reset();
    case SERIAL_MISC_SET_SETTINGS:
        return misc_set_settings(cmd.params);
    case SERIAL_MISC_GET_SETTINGS:
        return misc_get_settings();
    default:
        std::string msg = "Command " + std::to_string(cmd.id) + " not found.";
        return msg;
    }
}

void set_calibration_inertial(const std::vector<float> &params,
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

void set_calibration_magnetic(const std::vector<float> &params,
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
std::string mag_set_calib(std::vector<float> params) {
    SerialOutputMode = SERIAL_PRINT_NOTHING;

    set_calibration_magnetic(params, soft_iron, hard_iron);
    kv_store_save_calibration("MagOffset", hard_iron);
    kv_store_save_calibration("MagGain", soft_iron);

    std::string s = int_to_hex(SERIAL_MAG_SET_CALIB) + ", Calibration set;";
    return s;
}

std::string mag_get_calib(void) {
    SerialOutputMode = SERIAL_PRINT_NOTHING;

    command_t cmd;
    cmd.id = SERIAL_MAG_GET_CALIB;
    get_calibration_magnetic(cmd, soft_iron, hard_iron);

    std::string msg = create_message(cmd);
    return msg;
}

std::string acc_set_calib(std::vector<float> params) {
    SerialOutputMode = SERIAL_PRINT_NOTHING;

    set_calibration_inertial(params, acc_misalignment, acc_gain, acc_offset);
    kv_store_save_calibration("AccOffset", acc_offset);
    kv_store_save_calibration("AccGain", acc_gain);

    std::string s = int_to_hex(SERIAL_ACC_SET_CALIB) + ", Calibration set;";
    return s;
}

std::string acc_get_calib(void) {
    SerialOutputMode = SERIAL_PRINT_NOTHING;

    command_t cmd;
    cmd.id = SERIAL_ACC_GET_CALIB;
    get_calibration_inertial(cmd, acc_misalignment, acc_gain, acc_offset);

    std::string msg = create_message(cmd);
    return msg;
}

std::string gyro_set_calib(std::vector<float> params) {
    SerialOutputMode = SERIAL_PRINT_NOTHING;

    set_calibration_inertial(params, gyro_misalignment, gyro_gain, gyro_offset);
    kv_store_save_calibration("GyroOffset", gyro_offset);
    kv_store_save_calibration("GyroGain", gyro_gain);

    std::string s = int_to_hex(SERIAL_GYRO_SET_CALIB) + ", Calibration set;";
    return s;
}

std::string gyro_get_calib(void) {
    SerialOutputMode = SERIAL_PRINT_NOTHING;

    command_t cmd;
    cmd.id = SERIAL_GYRO_GET_CALIB;
    get_calibration_inertial(cmd, gyro_misalignment, gyro_gain, gyro_offset);

    std::string msg = create_message(cmd);
    return msg;
}

std::string set_print_mode(std::vector<float> params) {
    if (params.size() < 1) {
        string err("Error: No serial output mode given");
        return err;
    }

    SerialOutputMode = static_cast<uint8_t>(params[0]);
    std::string msg =
        "Output mode set to " + int_to_hex(SerialOutputMode) + ";";
    return msg;
}

std::string misc_set_settings(std::vector<float> params) {
    SerialOutputMode = SERIAL_PRINT_NOTHING;

    if (params.size() < 7) {
        string err("Error: Not enough parameters given");
        return err;
    }
    AxisOffset.axis.x = params[0]; // Yaw
    AxisOffset.axis.y = params[1]; // Pitch
    AxisOffset.axis.z = params[2]; // Roll
    //
    AHRSsettings.gain = params[3];
    AHRSsettings.accelerationRejection = params[4];
    AHRSsettings.magneticRejection = params[5];
    AHRSsettings.recoveryTriggerPeriod = params[6];

    FusionAhrsSetSettings(&AHRS, &AHRSsettings);
    FusionAhrsReset(&AHRS);

    std::string msg = "AHRS settings set;";
    return msg;
}

std::string misc_get_settings(void) {
    SerialOutputMode = SERIAL_PRINT_NOTHING;

    command_t cmd;
    cmd.id = SERIAL_MISC_GET_SETTINGS;
    cmd.n_params = 7;

    cmd.params.push_back(AxisOffset.axis.x);
    cmd.params.push_back(AxisOffset.axis.y);
    cmd.params.push_back(AxisOffset.axis.z);

    cmd.params.push_back(AHRSsettings.gain);
    cmd.params.push_back(AHRSsettings.accelerationRejection);
    cmd.params.push_back(AHRSsettings.magneticRejection);
    cmd.params.push_back(AHRSsettings.recoveryTriggerPeriod);

    std::string msg = create_message(cmd);
    return msg;
}

// TODO: Stupid wrapper function...
std::string _kv_store_reset() {
    kv_store_reset();

    std::string msg("Resetting KV-store");
    return msg;
}

/*
-------------------- END OF SERIAL INPUT PART --------------------
*/
