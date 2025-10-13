#ifndef __KV_STORAGE__

#include "Fusion/FusionMath.h"
#include <map>
#include <string>

using std::string;

// INFO: Use the enum when accessing kv_keys. This should guard againts typos
// when accessing kv-storage.
enum {
    cal_mag_offset,
    cal_mag_gain,
    cal_acc_offset,
    cal_acc_gain,
    cal_gyro_offset,
    cal_gyro_gain,
    cal_euler_output_offset,
    cal_ahrs_settings
};

extern std::map<int, std::string> kv_keys;

bool kv_store_initialized(void);

/*
 * Save calibration to kv-store.
 *
 * Returns true if saved successfully.
 * Returns false if kv-store returned an error.
 */
bool kv_store_save_calibration(const std::string key, const FusionVector &data);

/*
  Load calibration values from the KVstore. If the KVStore key does not
  exist, set calibration to given factory defaults.

  Returns true if calibration was loaded from KVStore succesfully.
  Returns false if loading failed and defaults were used instead.
*/
bool kv_store_load_calibration(const std::string key, FusionVector &calib,
                               FusionVector &factory_default);

/*
 * Same as above overloaded for FUsionMatrix.
 */
bool kv_store_save_calibration(const std::string key, const FusionMatrix &data);
bool kv_store_load_calibration(const std::string key, FusionMatrix &calib,
                               FusionMatrix &factory_default);

void kv_store_reset(void);

#endif // !f __KV_STORAGE__
