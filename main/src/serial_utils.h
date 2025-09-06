/*
 * Functions for creating messages for binary transmissions.
 */
#ifndef __SERIAL_UTILS__
#define __SERIAL_UTILS__

#include <stdint.h>
#include <string>
#include <tuple>
#include <vector>

using std::string;
using std::tuple;

typedef struct {
    int8_t id;
    int8_t n_params;
    std::vector<float> params;
    std::string err;
} command_t;

enum {
    ASCII_NUL = 0x00,
    ASCII_SOH = 0x01,
    ASCII_STX = 0x02,
    ASCII_ETX = 0x03,
    ASCII_EOT = 0x04,
    ASCII_ESC = 0x1b,
    ASCII_LF = 0x0a,
    ASCII_CR = 0x0d,
    ESCAPE_OFFSET = 0x20,
};

/*
 * Create header and body from command.
 */
std::tuple<std::string, std::string> command2bytes(const command_t &cmd);

/*
 * Create command from supplied header and body.
 */
command_t bytes2command(const std::string &header, const std::string &body);

std::string create_message(const command_t &cmd);
command_t retrieve_command(const std::string &msg);

/*
 * Check that the message has all the necessary control characters in the
 * correct order.
 */
int sanity_check_message(const std::string &msg);

/*
 * Extract the header and body from message following form described
 * in 'create_message'.
 */
tuple<std::string, std::string>
retrieve_header_and_body(const std::string &msg);

/*
 * Preface all characters found in the list TRANSMISSION_CONTROL_CHARS with
 * the escape byte ASCII_ESC and add ESCAPE_OFFSET to the byte.
 *
 * Example: \x04\x3f => \x1b\x24\x3f
 */
std::string parse_outbound_bytes(const std::string &msg);

/*
 * Remove all ASCII_ESC bytes from the message buffer and remove the
 * ESCAPE_OFFSET from the next byte.
 *
 * Example: \x1b\x24\x3f => \x04\x3f
 */
std::string parse_inbound_bytes(const std::string &msg);

/*
 * Helper function for checking byte values.
 */
void print_hex(const std::string &msg);

#endif // __SERIAL_UTILS_
