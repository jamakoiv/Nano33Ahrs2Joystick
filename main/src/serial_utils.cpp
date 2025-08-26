#include <Arduino.h>
#include <cstring>
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

// INFO: Technically undefined behaviour, but works on GCC-suite (and clang).
union float_bytes_t {
    float f;
    char b[sizeof(float)];
};

std::tuple<string, string> command2bytes(const command_t &cmd) {
    string header;
    float_bytes_t float_bytes;

    header.reserve(3);
    header.push_back(cmd.id);
    header.push_back(cmd.n_params);

    // TODO: No endianess handling, will break on some platform combinations.
    string body;
    body.reserve(cmd.params.size() * sizeof(float));
    for (float f : cmd.params) {
        float_bytes.f = f;
        body.append(float_bytes.b, sizeof(float));
    }

    return std::make_tuple(header, body);
}

command_t bytes2command(const string &header, const string &body) {
    command_t cmd;
    float_bytes_t float_bytes;

    cmd.id = header[0];
    cmd.n_params = header[1];

    cmd.params.reserve(cmd.n_params);
    const char *tmp_array = body.c_str();
    for (int i = 0; i < cmd.n_params; i++, tmp_array += sizeof(float)) {
        std::memcpy(float_bytes.b, tmp_array, sizeof(float));
        cmd.params.push_back(float_bytes.f);
    }

    return cmd;
}

string create_message(const command_t &cmd) {
    auto [raw_header, raw_body] = command2bytes(cmd);
    string header = parse_outbound_bytes(raw_header);
    string body = parse_outbound_bytes(raw_body);

    string msg = string(1, SOH) + header + string(1, STX) + body +
                 string(1, ETX) + string(1, EOT);
    return msg;
}

command_t retrieve_command(const string &msg) {
    if (sanity_check_message(msg) != 0) {
        return command_t{0, 0};
    }

    auto [raw_header, raw_body] = retrieve_header_and_body(msg);

    Serial.print("[retrieve_command] raw_header: ");
    Serial.println(raw_header.c_str());
    Serial.print("[retrieve_command] raw_body: ");
    Serial.println(raw_body.c_str());

    string header = parse_inbound_bytes(raw_header);
    string body = parse_inbound_bytes(raw_body);

    Serial.print("[retrieve_command] header: ");
    Serial.println(header.c_str());
    Serial.print("[retrieve_command] body: ");
    Serial.println(body.c_str());

    command_t cmd = bytes2command(header, body);
    return cmd;
}

// string create_message(const string &header, const string &body) {
//     string msg = string(1, SOH) + header + string(1, STX) + body +
//                  string(1, ETX) + string(1, EOT);
//
//     return msg;
// }

std::tuple<string, string> retrieve_header_and_body(const string &msg) {
    size_t SOH_pos = msg.find(SOH);
    size_t STX_pos = msg.find(STX);
    size_t ETX_pos = msg.find(ETX);
    size_t EOT_pos = msg.find(EOT);

    string header = msg.substr(SOH_pos + 1, STX_pos - SOH_pos - 1);
    string body = msg.substr(STX_pos + 1, ETX_pos - STX_pos - 1);

    Serial.print("[retrieve_header_and_body] header: ");
    Serial.println(header.c_str());
    Serial.print("[retrieve_header_and_body] body: ");
    Serial.println(body.c_str());

    return std::make_tuple(header, body);
}

int sanity_check_message(const string &msg) {
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

string parse_outbound_bytes(const string &msg) {
    string res;
    res.reserve(msg.length() + 20); // TODO: Make a better estimate.

    for (char msg_byte : msg) {
        for (char ctrl_byte : TRANSMISSION_CONTROL_CHARS) {
            if (msg_byte == ctrl_byte) {
                res.push_back(ESC);
                res.push_back(msg_byte + ESCAPE_OFFSET);
                goto next_loop;
            }
        }
        res.push_back(msg_byte);
    next_loop:
        continue;
    }

    return res;
}

string parse_inbound_bytes(const string &msg) {
    string res;
    res.reserve(msg.length());

    bool escape_found = false;
    for (char msg_byte : msg) {
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

    return res;
}

void print_hex(const string &msg) {
    for (const auto &c : msg) {
        std::cout << std::hex << std::setw(2) << std::setfill('0')
                  << (static_cast<int>(c) & 0xff) << " ";
    }
    std::cout << std::dec << std::endl;
}
