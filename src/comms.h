#include <string>
#include <vector>

float remap_yaw(float yaw, float d);

bool serial_handshake(void);
std::string read_serial_input(void);
void execute_command(std::vector<std::string> params);
void check_serial_input(void);

void printAHRSeuler(void);
void printNothing(void);
void printAccCalib(void);
void printMagCalib(void);
void printGyroCalib(void);
void printAccRaw(void);
void printMagRaw(void);
void printGyroRaw(void);
void print_output(void);

void mag_set_calib(std::string input);
void mag_get_calib(std::string input);
void acc_set_calib(std::string input);
void acc_get_calib(std::string input);
void gyro_set_calib(std::string input);
void gyro_get_calib(std::string input);

void set_print_nothing(std::string input);
void set_print_ahrs(std::string input);
void set_print_mag_raw(std::string input);
void set_print_mag_calib(std::string input);
void set_print_acc_raw(std::string input);
void set_print_acc_calib(std::string input);
void set_print_gyro_raw(std::string input);
void set_print_gyro_calib(std::string input);

void kv_store_reset(std::string input);
