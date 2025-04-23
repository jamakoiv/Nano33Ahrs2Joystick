#ifndef __KV_STORAGE__

#include "MyVector/MyVector.h"
#include "ino_globals.h"

#include <string>

using MyVector::vector;

bool kv_store_initialized(void);
bool kv_store_save_calibration(const std::string &key, const vector &data);
bool kv_store_load_calibration(const std::string &key, vector &calib,
                               vector &factory_default);
void kv_store_reset(std::string input);

#endif // !f __KV_STORAGE__
