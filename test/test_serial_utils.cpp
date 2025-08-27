#include "../main/src/serial_utils.h"
#include <cassert>
#include <iostream>

void test_parse_outbound_bytes() {
    string test_msg{0x01, 'a', 'b', 0x02, 'c', 'd', 0x03, 'e', 'f', 0x04};

    string correct{0x1b, 0x21, 'a',  'b', 0x1b, 0x22, 'c',
                   'd',  0x1b, 0x23, 'e', 'f',  0x1b, 0x24};

    print_hex(test_msg);
    print_hex(correct);

    string res = parse_outbound_bytes(test_msg);
    print_hex(res);
    assert(correct == res);
}

void test_parse_inbound_bytes() {
    string test_msg{0x1b, 0x21, 'a',  'b', 0x1b, 0x22, 'c',
                    'd',  0x1b, 0x23, 'e', 'f',  0x1b, 0x24};

    string correct{0x01, 'a', 'b', 0x02, 'c', 'd', 0x03, 'e', 'f', 0x04};

    print_hex(test_msg);
    print_hex(correct);

    string res = parse_inbound_bytes(test_msg);
    print_hex(res);
    assert(correct == res);
}

void test_retrieve_header_and_body() {
    string msg{0x01, 0x31, 0x05, 0x02, 0x20, 0x21,
               0x22, 0x23, 0x24, 0x03, 0x04};

    string correct_header{0x31, 0x05};
    string correct_body{0x20, 0x21, 0x22, 0x23, 0x24};

    auto [header, body] = retrieve_header_and_body(msg);

    print_hex(header);
    print_hex(correct_header);
    assert(header == correct_header);

    print_hex(body);
    print_hex(correct_body);
    assert(body == correct_body);
}

void test_sanity_check_message() {
    string correct{0x01, 0x31, 0x05, 0x02, 0x20, 0x21,
                   0x22, 0x23, 0x24, 0x03, 0x04};

    int res = sanity_check_message(correct);
    assert(res == 0);
}

void test_sanity_check_message_missing_SOH() {
    string fail{0x31, 0x05, 0x02, 0x20, 0x21, 0x22, 0x23, 0x24, 0x03, 0x04};

    int res = sanity_check_message(fail);
    assert(res == -10);
}

void test_sanity_check_message_missing_STX() {
    string correct{0x01, 0x31, 0x05, 0x20, 0x21, 0x22, 0x23, 0x24, 0x03, 0x04};

    int res = sanity_check_message(correct);
    assert(res == -11);
}

void test_sanity_check_message_missing_ETX() {
    string correct{0x01, 0x31, 0x05, 0x02, 0x20, 0x21, 0x22, 0x23, 0x24, 0x04};

    int res = sanity_check_message(correct);
    assert(res == -12);
}

void test_sanity_check_message_missing_EOT() {
    string correct{0x01, 0x31, 0x05, 0x02, 0x20, 0x21, 0x22, 0x23, 0x24, 0x03};

    int res = sanity_check_message(correct);
    assert(res == -13);
}

void test_sanity_check_message_wrong_order() {
    string fail{0x02, 0x31, 0x05, 0x01, 0x20, 0x21,
                0x22, 0x23, 0x24, 0x03, 0x04};

    int res = sanity_check_message(fail);
    assert(res == -2);
}

void test_command2bytes() {
    command_t cmd;
    cmd.id = 0x31;
    cmd.n_params = 3;
    cmd.params = {1.11, 2.22, 3.33};

    std::string correct_header{0x31, 0x03};
    std::string correct_body(
        "\x7b\x14\x8e\x3f\x7b\x14\x0e\x40\xb8\x1e\x55\x40");

    auto [header, body] = command2bytes(cmd);

    assert(header == correct_header);
    assert(body == correct_body);

    print_hex(correct_header);
    print_hex(header);
    print_hex(correct_body);
    print_hex(body);
}

void test_bytes2command() {
    std::string header("\x31\x03");
    std::string body("\x7b\x14\x8e\x3f\x7b\x14\x0e\x40\xb8\x1e\x55\x40");

    command_t correct_cmd;
    correct_cmd.id = 0x31;
    correct_cmd.n_params = 3;
    correct_cmd.params = {1.11, 2.22, 3.33};

    command_t res = bytes2command(header, body);

    assert(res.id == correct_cmd.id);
    assert(res.n_params == correct_cmd.n_params);

    for (float f : res.params) {
        std::cout << f << std::endl;
    }

    assert(res.params.size() == correct_cmd.params.size());
    assert(res.params[0] == correct_cmd.params[0]);
    assert(res.params[1] == correct_cmd.params[1]);
    assert(res.params[2] == correct_cmd.params[2]);
}

void test_create_message() {
    // INFO: Test without ESCAPE-bytes.
    command_t cmd;
    cmd.id = 0x31;
    cmd.n_params = 5;
    cmd.params = {1.11, 1.11, 1.11, 1.11, 1.11};

    string correct("\x01\x31\x05\x02\x7b\x14\x8e\x3f\x7b\x14\x8e\x3f\x7b\x14"
                   "\x8e\x3f\x7b\x14\x8e\x3f\x7b\x14\x8e\x3f\x03\x04");
    string res = create_message(cmd);

    std::cout << "[test_create_message] correct: ";
    print_hex(correct);
    std::cout << "[test_create_message] create_message: ";
    print_hex(res);

    assert(correct == res);
}

void test_create_message_escape() {
    // INFO: Test with ESCAPE-bytes.
    command_t cmd;
    cmd.id = 0x02;
    cmd.n_params = 3;
    cmd.params = {0.5117493, 1.11, 1.11};

    string correct("\x01\x1b\x22\x1b\x23\x02\x1b\x21\x1b\x22\x1b\x23\x3f\x7b"
                   "\x14\x8e\x3f\x7b\x14\x8e\x3f\x03\x04");
    string res = create_message(cmd);

    std::cout << "[test_create_message_escape] correct: ";
    print_hex(correct);
    std::cout << "[test_create_message_escape] create_message: ";
    print_hex(res);

    assert(correct == res);
}

void test_retrieve_command() {
    string msg("\x01\x31\x05\x02\x7b\x14\x8e\x3f\x7b\x14\x8e\x3f\x7b\x14"
               "\x8e\x3f\x7b\x14\x8e\x3f\x7b\x14\x8e\x3f\x03\x04");

    command_t correct_cmd = {0x31, 5, {1.11, 1.11, 1.11, 1.11, 1.11}};
    command_t res = retrieve_command(msg);

    assert(res.id == correct_cmd.id);
    assert(res.n_params == correct_cmd.n_params);
    assert(res.params == std::vector<float>({1.11, 1.11, 1.11, 1.11, 1.11}));
}

void test_retrieve_command_escape() {
    string msg("\x01\x1b\x22\x1b\x23\x02\x1b\x21\x1b\x22\x1b\x23\x3f\x7b"
               "\x14\x8e\x3f\x7b\x14\x8e\x3f\x03\x04");

    command_t correct_cmd = {0x02, 3, {0.5117493, 1.11, 1.11}};
    command_t res = retrieve_command(msg);

    assert(res.id == correct_cmd.id);
    assert(res.n_params == correct_cmd.n_params);
    assert(res.params == std::vector<float>({0.5117493, 1.11, 1.11}));
}

int main() {
    test_parse_inbound_bytes();
    test_parse_outbound_bytes();

    test_retrieve_header_and_body();

    test_sanity_check_message();
    test_sanity_check_message_missing_SOH();
    test_sanity_check_message_missing_STX();
    test_sanity_check_message_missing_ETX();
    test_sanity_check_message_missing_EOT();
    test_sanity_check_message_wrong_order();

    test_command2bytes();
    test_bytes2command();

    test_create_message();
    test_create_message_escape();

    test_retrieve_command();
    test_retrieve_command_escape();
}
