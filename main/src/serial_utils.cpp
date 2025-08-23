#include <iomanip>
#include <iostream>
#include <set>
#include <string>

#include "serial_utils.h"

using std::string;

enum ASCII {
    SOH = 0x01,
    STX = 0x02,
    ETX = 0x03,
    EOT = 0x04,
    ESC = 0x1b,
    ESCAPE_OFFSET = 0x20,
};
std::set<char> TRANSMISSION_CONTROL_CHARS = {SOH, STX, ETX, EOT, ESC};

/*
 * Create message in form <SOH><header><STX><body><ETX><EOT>.
 */
string create_message(string header, string body) {

    string msg = string(1, SOH) + header + string(1, STX) + body +
                 string(1, ETX) + string(1, EOT);

    return msg;
}

std::tuple<string, string> retrieve_header_and_body(string msg) {
    size_t SOH_pos = msg.find(SOH);
    size_t STX_pos = msg.find(STX);
    size_t ETX_pos = msg.find(ETX);
    size_t EOT_pos = msg.find(EOT);

    string header = msg.substr(SOH_pos + 1, STX_pos - SOH_pos - 1);
    string body = msg.substr(STX_pos + 1, ETX_pos - STX_pos - 1);

    return std::make_tuple(header, body);
}

/*
 * Check that the message has all the necessary control characters in the
 * correct order.
 */
int sanity_check_message(string msg) {

    size_t SOH_pos = msg.find(SOH);
    size_t STX_pos = msg.find(STX);
    size_t ETX_pos = msg.find(ETX);
    size_t EOT_pos = msg.find(EOT);

    if (SOH_pos == string::npos || STX_pos == string::npos ||
        ETX_pos == string::npos || EOT_pos == string::npos) {
        // std::cerr << "Message missing control characters" << std::endl;
        return -1;
    }

    if (SOH_pos > STX_pos || STX_pos > ETX_pos || ETX_pos > EOT_pos) {
        // std::cerr << "Message control characters out of order" << std::endl;
        return -2;
    }

    return 0;
}

/*
 * Preface all characters found in the list TRANSMISSION_CONTROL_CHARS with the
 * escape byte ASCII.ESC and add ESCAPE_OFFSET to the byte.
 *
 * Example: \x04\x3f => \x1b\x24\x3f
 */
int parse_outbound_bytes(string msg, string &res) {
    res.clear();
    res.reserve(msg.length() + 20);

    for (auto msg_byte : msg) {
        for (auto ctrl_byte : TRANSMISSION_CONTROL_CHARS) {
            if (msg_byte == ctrl_byte) {
                res.push_back(ESC);
                res.push_back(msg_byte + ESCAPE_OFFSET);
                goto next_loop;
            }
        }
        res.push_back(msg_byte);
    next_loop:
    }

    return 0;
}

/*
 * Remove all ASCII.ESC bytes from the message buffer and remove the
 * ESCAPE_OFFSET from the next byte.
 *
 * Example: \x1b\x24\x3f => \x04\x3f
 */
int parse_inbound_bytes(string msg, string &res) {
    res.clear();
    res.reserve(msg.length());

    bool escape_found;
    for (auto msg_byte : msg) {
        if (msg_byte == ESC) {
            escape_found = true;
            continue;
        } else if (escape_found) {
            res.push_back(msg_byte - ESCAPE_OFFSET);
            escape_found = false;
        } else {
            res.push_back(msg_byte);
        }
    }

    return 0;
}

/*
 * Helper function for checking byte values.
 */
void print_hex(const string &msg) {
    for (const auto &c : msg) {
        std::cout << std::hex << std::setw(2) << std::setfill('0')
                  << (static_cast<int>(c) & 0xff) << " ";
    }
    std::cout << std::dec << std::endl;
}
