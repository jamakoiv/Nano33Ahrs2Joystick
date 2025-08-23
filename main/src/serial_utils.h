#include <string>

using std::string;

/*
 *
 */

/*
 * Create message in form <SOH><header><STX><body><ETX><EOT>.
 */
string create_message(string header, string body);

/*
 * Check that the message has all the necessary control characters in the
 * correct order.
 */
int sanity_check_message(string msg);

/*
 *
 */
std::tuple<string, string> retrieve_header_and_body(string msg);

/*
 * Preface all characters found in the list TRANSMISSION_CONTROL_CHARS with
 * the escape byte ASCII_ESC and add ESCAPE_OFFSET to the byte.
 *
 * Example: \x04\x3f => \x1b\x24\x3f
 */
int parse_outbound_bytes(string msg, string &res);

/*
 * Remove all ASCII_ESC bytes from the message buffer and remove the
 * ESCAPE_OFFSET from the next byte.
 *
 * Example: \x1b\x24\x3f => \x04\x3f
 */
int parse_inbound_bytes(string msg, string &res);

/*
 * Helper function for checking byte values.
 */
void print_hex(const string &msg);
