#include <Arduino.h>
#include <kvstore_global_api.h>
#include <mbed_error.h>
#include <stdint.h>
#include <string>

#include "Fusion/FusionMath.h"
#include "ino_globals.h"
#include "kv_storage.h"
#include "utils.h"

std::string kv_keys[7] = {"MagOffset",  "MagGain",  "AccOffset",   "AccGain",
                          "GyroOffset", "GyroGain", "OutputOffset"};

/*
"AxisOffset"-------------------- KV-STORE PART --------------------
*/

const uint8_t KV_BUFFER_SIZE = 64;
const std::string kv_path = "/kv/";
const std::string kv_init_key = kv_path + "KVStore_global_api_init";

bool kv_store_initialized(void) {
    kv_info_t info;

    auto res = kv_get_info(kv_init_key.c_str(), &info);

    if (res == MBED_ERROR_ITEM_NOT_FOUND) {
        Serial.println("KVStore not initialized. Resetting KVStore...");
        res = kv_reset(kv_path.c_str());
        if (res != MBED_SUCCESS) {
            return false;
        }

        res = kv_set(kv_init_key.c_str(), "1", 1, 0);
        if (res != MBED_SUCCESS) {
            Serial.println("Error setting init key to KVStore.");
            return false;
        }
        Serial.println("KVStore reset successfully.");
        return true;
    } else {
        Serial.println("KVStore seems ok...");
        return true;
    }
}

bool kv_store_save_calibration(const std::string key,
                               const FusionVector &data) {
    std::string data_str = std::to_string(data.axis.x) + "," +
                           std::to_string(data.axis.y) + "," +
                           std::to_string(data.axis.z);
    auto full_key = kv_path + key;

    auto res = kv_set(full_key.c_str(), data_str.c_str(), data_str.length(), 0);

    if (res == MBED_SUCCESS)
        return true;
    else
        return false;
}

bool kv_store_load_calibration(const std::string key, FusionVector &calib,
                               FusionVector &factory_default) {
    /*
      Load calibration values from the KVstore. If the KVStore key does not
      exist, set calibration to given factory defaults.

      Returns true if calibration was loaded from KVStore succesfully.
      Returns false if loading failed and defaults were used instead.
    */
    static char kv_get_buffer[KV_BUFFER_SIZE];
    static std::string VALUES_DELIMITER = ",";

    size_t bytes_received;
    kv_info_t info;
    auto full_key = kv_path + key;
    calib = factory_default;

    auto res = kv_get_info(full_key.c_str(), &info);
    if (res == MBED_ERROR_ITEM_NOT_FOUND)
        return false;

    res = kv_get(full_key.c_str(), kv_get_buffer, info.size, &bytes_received);
    if (res != MBED_SUCCESS)
        return false;

    auto values =
        split_and_strtof(std::string(kv_get_buffer), VALUES_DELIMITER);
    if (values.size() != 3)
        return false;

    calib = (FusionVector){values[0], values[1], values[2]};
    return true;
}

/* Brutal cut-paste for FusionMatrix overloads */
bool kv_store_save_calibration(const std::string key,
                               const FusionMatrix &data) {
    std::string data_str = std::to_string(data.element.xx) + "," +
                           std::to_string(data.element.yy) + "," +
                           std::to_string(data.element.zz);

    auto full_key = kv_path + key;

    auto res = kv_set(full_key.c_str(), data_str.c_str(), data_str.length(), 0);
    if (res == MBED_SUCCESS)
        return true;
    else
        return false;
}

bool kv_store_load_calibration(const std::string key, FusionMatrix &calib,
                               FusionMatrix &factory_default) {
    /*
      Load calibration values from the KVstore. If the KVStore key does not
      exist, set calibration to given factory defaults.

      Returns true if calibration was loaded from KVStore succesfully.
      Returns false if loading failed and defaults were used instead.
    */
    static char kv_get_buffer[KV_BUFFER_SIZE];
    static std::string VALUES_DELIMITER = ",";

    size_t bytes_received;
    kv_info_t info;
    auto full_key = kv_path + key;

    calib = factory_default;

    auto res = kv_get_info(full_key.c_str(), &info);
    if (res == MBED_ERROR_ITEM_NOT_FOUND)
        return false;

    res = kv_get(full_key.c_str(), kv_get_buffer, info.size, &bytes_received);
    if (res != MBED_SUCCESS)
        return false;

    auto values =
        split_and_strtof(std::string(kv_get_buffer), VALUES_DELIMITER);
    if (values.size() != 3)
        return false;

    calib.element.xx = values[0];
    calib.element.yy = values[1];
    calib.element.zz = values[2];

    return true;
}

void kv_store_reset(void) {
    // TODO: kv_store_initialized calls kv_reset as well.
    kv_reset(kv_path.c_str());

    if (!kv_store_initialized()) {
        while (true) {
            Serial.println("Unrecoverable error resetting KVStore.");
            delay(2000);
        }
    }

    kv_store_save_calibration(kv_keys[cal_mag_offset], hard_iron_default);
    kv_store_save_calibration(kv_keys[cal_mag_gain], soft_iron_default);
    kv_store_save_calibration(kv_keys[cal_acc_offset], acc_offset_default);
    kv_store_save_calibration(kv_keys[cal_acc_gain], acc_gain_default);
    kv_store_save_calibration(kv_keys[cal_gyro_offset], gyro_offset_default);
    kv_store_save_calibration(kv_keys[cal_gyro_gain], gyro_gain_default);
    kv_store_save_calibration(kv_keys[cal_euler_output_offset],
                              AxisOffset_default);

    hard_iron = hard_iron_default;
    soft_iron = soft_iron_default;
    acc_offset = acc_offset_default;
    acc_gain = acc_gain_default;
    gyro_offset = gyro_offset_default;
    gyro_gain = gyro_gain_default;
    AxisOffset = AxisOffset_default;
}
/*
-------------------- END OF KV-STORE PART --------------------
*/
