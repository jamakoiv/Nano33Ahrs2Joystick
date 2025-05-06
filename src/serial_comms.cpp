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

using input_func_ptr_t = void (*)(std::string input);
std::map<uint8_t, input_func_ptr_t> input_functions = {
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

void execute_command(std::vector<std::string> params) {
    // Guard against inputs which would crash the program at the
    // "(it->second)(params[1])"
    if (params.size() == 1)
        params.emplace_back(" ");

    Serial.print("params[0]: ");
    Serial.println(params[0].c_str());

    uint32_t command =
        static_cast<uint8_t>(strtol(params[0].c_str(), NULL, 16));

    auto it = input_functions.find(command);
    if (it == input_functions.end()) {
        Serial.print("Command not found: 0x");
        Serial.println(command, HEX);
    } else {
        (it->second)(params[1]);
    }
}

typedef struct command_t {
    int id;
    std::vector<float> params;
};

void check_serial_input(void) {
    /*
      Check for commands in serial.
    */
    static const std::string COMMAND_DELIMITER = ";";
    static const std::string PARAMETER_DELIMITER = ",";

    std::strncpy(serialBuffer, NULL, SERIAL_READ_BUFFER_SIZE);
    std::vector<std::string> lines;
    std::vector<command_t> commands;

    while (Serial.readBytesUntil(';', serialBuffer, SERIAL_READ_BUFFER_SIZE)) {
        lines.push_back(serialBuffer);
        std::strncpy(serialBuffer, NULL, SERIAL_READ_BUFFER_SIZE);
    }

    for (std::string s : lines) {
        std::vector<std::string> cmd_tokens = split(s, ',');
        std::vector<float> cmd_params;

        int cmd_id = std::strtol(cmd_tokens[0].c_str(), NULL, 16);
        cmd_tokens.erase(cmd_tokens.begin());
        for (std::string token : cmd_tokens) {
            cmd_params.push_back(std::strtof(token.c_str(), NULL));
        }
        commands.push_back({cmd_id, cmd_params});
    }

    for (command_t cmd : commands) {
        Serial.print("\ncmd: ");
        Serial.print(cmd.id);
        Serial.print(", params: ");

        for (float p : cmd.params) {
            Serial.print(p);
            Serial.print(", ");
        }
    }

    delay(5000); // Small delay in case the received message is
                 // incomplete when we check Serial.available.
                 // Make larger if you want to send data manually via terminal.

    // auto input = read_serial_input();
    // auto input_params = split_input(input, OPTIONS_DELIMITER);
    // execute_command(input_params);
}

void mag_set_calib(std::string input) {
    // set_calib_helper(split_and_strtof(input, ","), MagOffset, MagGain);
    kv_store_save_calibration("MagOffset", MagOffset);
    kv_store_save_calibration("MagGain", MagGain);
    Serial.println("Calibration set.");
}

void mag_get_calib(std::string input) {
    SerialOutputMode = SERIAL_PRINT_NOTHING;
    std::string str = "Magnetic offset (x,y,z): " + MagOffset.to_string() +
                      "; Magnetic gain (x,y,z): " + MagGain.to_string();
    Serial.println(str.c_str());
}

void acc_set_calib(std::string input) {
    set_calib_helper(split_and_strtof(input, ","), AccOffset, AccGain);
    kv_store_save_calibration("AccOffset", AccOffset);
    kv_store_save_calibration("AccGain", AccGain);
    Serial.println("Calibration set.");
}

void acc_get_calib(std::string input) {
    SerialOutputMode = SERIAL_PRINT_NOTHING;
    std::string str = "Accelerometer offset (x,y,z): " + AccOffset.to_string();
    +"; Accelerometer gain (x,y,z): " + AccGain.to_string();
    Serial.println(str.c_str());
}

void gyro_set_calib(std::string input) {
    set_calib_helper(split_and_strtof(input, ","), GyroOffset, GyroGain);
    kv_store_save_calibration("GyroOffset", GyroOffset);
    kv_store_save_calibration("GyroGain", GyroGain);
    Serial.println("Calibration set.");
}

void gyro_get_calib(std::string input) {
    SerialOutputMode = SERIAL_PRINT_NOTHING;
    std::string str = "Gyroscope offset (x,y,z): " + GyroOffset.to_string();
    +"; Gyroscope gain (x,y,z): " + GyroGain.to_string();
    Serial.println(str.c_str());
}

void yaw_set_offset(std::string input) {
    set_calib_helper(split_and_strtof(input, ","), AxisOffset);
    kv_store_save_calibration("AxisOffset", AxisOffset);
    Serial.println("Calibration set.");
}

void yaw_get_offset(std::string input) {
    SerialOutputMode = SERIAL_PRINT_NOTHING;
    std::string str = "Axis Offset (yaw,pitch,roll): " + AxisOffset.to_string();
    Serial.println(str.c_str());
}

/* functions for settings print output mode. */
void set_print_nothing(std::string input) {
    SerialOutputMode = SERIAL_PRINT_NOTHING;
    Serial.println("Output-mode set to NOTHING.");
}
void set_print_ahrs(std::string input) {
    SerialOutputMode = SERIAL_PRINT_AHRS;
    Serial.println("Output-mode set to AHRS.");
}
void set_print_mag_raw(std::string input) {
    SerialOutputMode = SERIAL_PRINT_MAG_RAW;
    Serial.println("Output-mode set to MAGNETOMETER-RAW.");
}
void set_print_mag_calib(std::string input) {
    SerialOutputMode = SERIAL_PRINT_MAG_CALIB;
    Serial.println("Output-mode set to MAGNETOMETER-CALIBRATED.");
}
void set_print_acc_raw(std::string input) {
    SerialOutputMode = SERIAL_PRINT_ACC_RAW;
    Serial.println("Output-mode set to ACCELEROMETER-RAW.");
}
void set_print_acc_calib(std::string input) {
    SerialOutputMode = SERIAL_PRINT_ACC_CALIB;
    Serial.println("Output-mode set to ACCELEROMETER-CALIBRATED.");
}
void set_print_gyro_raw(std::string input) {
    SerialOutputMode = SERIAL_PRINT_GYRO_RAW;
    Serial.println("Output-mode set to GYROSCOPE-RAW.");
}
void set_print_gyro_calib(std::string input) {
    SerialOutputMode = SERIAL_PRINT_GYRO_CALIB;
    Serial.println("Output-mode set to GYROSCOPE-RAW.");
}
/*
-------------------- END OF SERIAL INPUT PART --------------------
*/
