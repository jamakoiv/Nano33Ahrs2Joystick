#ifndef __SERIAL_UTILS__
#define __SERIAL_UTILS__

#include <stdint.h>
#include <string>
#include <tuple>
#include <vector>

using std::string;
using std::tuple;

typedef struct {
    uint8_t id;
    uint8_t n_bytes;
    std::vector<float> params;
} command_t;

/*
 * Functions for creating messages for binary transmissions.
 *
 * Sending messages:
 *
 * auto [raw_header, raw_body] = command_2_strings(cmd);
 * string header = parse_outbound_bytes(raw_header);
 * string body = parse_outbound_bytes(raw_body);
 * string msg = create_message(header, body);
 * Serial.println(msg.c_str());
 *
 * Receiving messages:
 *
 * Serial.readline(stop_byte, buffer, n_max);
 * string msg(buffer);
 * if (sanity_check_message(msg) == 0) {
 *      print("Malformed message");
 * }
 * auto [header, body] = retrieve_header_and_body(msg);
 */

/*
 * Create header and body from command.
 */
std::tuple<string, string> command_2_strings(const command_t &cmd);

/*
 * Create command from supplied header and body.
 */
command_t strings_2_command(const string &header, const string &body);

/*
 * Create message in form <SOH><header><STX><body><ETX><EOT>.
 */
string create_message(const string &header, const string &body);

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
