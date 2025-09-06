// #include <Arduino.h>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <set>
#include <string>

#include "serial_utils.h"

using std::string;

std::set<char> TRANSMISSION_CONTROL_CHARS = {ASCII_SOH, ASCII_STX, ASCII_ETX,
                                             ASCII_EOT, ASCII_ESC, ASCII_NUL,
                                             ASCII_LF,  ASCII_CR};

// INFO: Technically undefined behaviour, but works on GCC-suite (and clang).
union float_bytes_t {
    float f;
    char b[sizeof(float)];
};

std::tuple<std::string, std::string> command2bytes(const command_t &cmd) {
    std::string header;
    float_bytes_t float_bytes;

    header.reserve(3);
    header.push_back(cmd.id);
    header.push_back(cmd.n_params);

    // TODO: No endianess handling, will break on some platform combinations.
    std::string body;
    body.reserve(cmd.params.size() * sizeof(float));
    for (float f : cmd.params) {
        float_bytes.f = f;
        body.append(float_bytes.b, sizeof(float));
    }

    return std::make_tuple(header, body);
}

command_t bytes2command(const std::string &header, const std::string &body) {
    command_t cmd;
    float_bytes_t float_bytes;

    cmd.id = header[0];
    cmd.n_params = header[1];

    cmd.params.reserve(cmd.n_params);
    const char *tmp_array = body.c_str();
    for (int i = 0; i < cmd.n_params; i++, tmp_array += sizeof(float)) {
        std::memcpy(float_bytes.b, tmp_array,
                    sizeof(float)); // TODO: Is memcpy the best we can do here?
        cmd.params.push_back(float_bytes.f);
    }

    return cmd;
}

std::string create_message(const command_t &cmd) {
    std::string raw_header, raw_body;
    std::tie(raw_header, raw_body) = command2bytes(cmd);
    std::string header = parse_outbound_bytes(raw_header);
    std::string body = parse_outbound_bytes(raw_body);

    std::string msg = std::string(1, ASCII_SOH) + header +
                      std::string(1, ASCII_STX) + body +
                      std::string(1, ASCII_ETX) + std::string(1, ASCII_EOT);
    return msg;
}

command_t retrieve_command(const std::string &msg) {
    int check = sanity_check_message(msg);
    if (check == -10) {
        return command_t{-10, 0, {}, "Header byte not found"};
    } else if (check == -11) {
        return command_t{-11, 0, {}, "Data start byte not found"};
    } else if (check == -12) {
        return command_t{-12, 0, {}, "Data end byte not found"};
    } else if (check == -13) {
        return command_t{-13, 0, {}, "Transmission end byte not found"};
    } else if (check == -2) {
        return command_t{-2, 0, {}, "Control characters in wrong order"};
    }

    std::string raw_header, raw_body;
    std::tie(raw_header, raw_body) = retrieve_header_and_body(msg);

    std::string header = parse_inbound_bytes(raw_header);
    std::string body = parse_inbound_bytes(raw_body);

    command_t cmd = bytes2command(header, body);
    return cmd;
}

std::tuple<std::string, std::string>
retrieve_header_and_body(const std::string &msg) {
    size_t SOH_pos = msg.find(ASCII_SOH);
    size_t STX_pos = msg.find(ASCII_STX);
    size_t ETX_pos = msg.find(ASCII_ETX);

    std::string header = msg.substr(SOH_pos + 1, STX_pos - SOH_pos - 1);
    std::string body = msg.substr(STX_pos + 1, ETX_pos - STX_pos - 1);

    // Serial.print("[retrieve_header_and_body] header: ");
    // Serial.println(header.c_str());
    // Serial.print("[retrieve_header_and_body] body: ");
    // Serial.println(body.c_str());

    return std::make_tuple(header, body);
}

int sanity_check_message(const std::string &msg) {
    size_t SOH_pos = msg.find(ASCII_SOH);
    size_t STX_pos = msg.find(ASCII_STX);
    size_t ETX_pos = msg.find(ASCII_ETX);
    size_t EOT_pos = msg.find(ASCII_EOT);

    if (SOH_pos == std::string::npos) {
        return -10;
    } else if (STX_pos == std::string::npos) {
        return -11;
    } else if (ETX_pos == std::string::npos) {
        return -12;
    } else if (EOT_pos == std::string::npos) {
        return -13;
    }

    if (SOH_pos > STX_pos || STX_pos > ETX_pos || ETX_pos > EOT_pos) {
        return -2;
    }

    return 0;
}

std::string parse_outbound_bytes(const std::string &msg) {
    std::string res;
    res.reserve(msg.length() + 20); // TODO: Make a better estimate.

    for (char msg_byte : msg) {
        for (char ctrl_byte : TRANSMISSION_CONTROL_CHARS) {
            if (msg_byte == ctrl_byte) {
                res.push_back(ASCII_ESC);
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

std::string parse_inbound_bytes(const std::string &msg) {
    std::string res;
    res.reserve(msg.length());

    bool escape_found = false;
    for (char msg_byte : msg) {
        if (msg_byte == ASCII_ESC) {
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

void print_hex(const std::string &msg) {
    for (const auto &c : msg) {
        std::cout << std::hex << std::setw(2) << std::setfill('0')
                  << (static_cast<int>(c) & 0xff) << " ";
    }
    std::cout << std::dec << std::endl;
}
