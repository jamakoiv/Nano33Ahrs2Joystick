#include <Arduino.h>
#include <kvstore_global_api.h>
#include <mbed_error.h>
#include <stdint.h>
#include <string>

#include "kv_storage.h"

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

bool kv_store_save_calibration(const std::string &key, const vector &data) {
    auto data_str = data.to_string();
    auto full_key = kv_path + key;

    auto res = kv_set(full_key.c_str(), data_str.c_str(), data_str.length(), 0);

    if (res == MBED_SUCCESS)
        return true;
    else
        return false;
}

bool kv_store_load_calibration(const std::string &key, vector &calib,
                               vector &factory_default) {
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

    calib = vector(values[0], values[1], values[2]);
    return true;
}

// kv_store_reset function has to match the 'std::map<...> input_functions'
// signature.
void kv_store_reset(std::string input) {

    // TODO: kv_store_initialized calls kv_reset as well.
    kv_reset(kv_path.c_str());

    if (!kv_store_initialized()) {
        while (true) {
            Serial.println("Unrecoverable error resetting KVStore.");
            delay(2000);
        }
    }

    kv_store_save_calibration(kv_keys[cal_mag_offset], MagOffset_default);
    kv_store_save_calibration(kv_keys[cal_mag_gain], MagGain_default);
    kv_store_save_calibration(kv_keys[cal_acc_offset], AccOffset_default);
    kv_store_save_calibration(kv_keys[cal_acc_gain], AccGain_default);
    kv_store_save_calibration(kv_keys[cal_gyro_offset], GyroOffset_default);
    kv_store_save_calibration(kv_keys[cal_gyro_gain], GyroGain_default);
    kv_store_save_calibration(kv_keys[cal_euler_output_offset],
                              AxisOffset_default);

    MagOffset = MagOffset_default;
    MagGain = MagGain_default;
    AccOffset = AccOffset_default;
    AccGain = AccGain_default;
    GyroOffset = GyroOffset_default;
    GyroGain = GyroGain_default;
    AxisOffset = AxisOffset_default;
}
/*
-------------------- END OF KV-STORE PART --------------------
*/
