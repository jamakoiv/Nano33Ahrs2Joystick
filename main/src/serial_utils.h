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

/*
 * Functions for creating messages for binary transmissions.
 */

/*
 * Create header and body from command.
 */
std::tuple<string, string> command2bytes(const command_t &cmd);

/*
 * Create command from supplied header and body.
 */
command_t bytes2command(const string &header, const string &body);

string create_message(const command_t &cmd);
command_t retrieve_command(const string &msg);

/*
 * Check that the message has all the necessary control characters in the
 * correct order.
 */
int sanity_check_message(const string &msg);

/*
 * Extract the header and body from message following form described
 * in 'create_message'.
 */
tuple<string, string> retrieve_header_and_body(const string &msg);

/*
 * Preface all characters found in the list TRANSMISSION_CONTROL_CHARS with
 * the escape byte ASCII_ESC and add ESCAPE_OFFSET to the byte.
 *
 * Example: \x04\x3f => \x1b\x24\x3f
 */
string parse_outbound_bytes(const string &msg);

/*
 * Remove all ASCII_ESC bytes from the message buffer and remove the
 * ESCAPE_OFFSET from the next byte.
 *
 * Example: \x1b\x24\x3f => \x04\x3f
 */
string parse_inbound_bytes(const string &msg);

/*
 * Helper function for checking byte values.
 */
void print_hex(const string &msg);

#endif // __SERIAL_UTILS_
