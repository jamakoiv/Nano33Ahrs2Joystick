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

void test_create_message() {
    string header{0x31, 0x05};
    string body{0x20, 0x21, 0x22, 0x23, 0x24};

    string correct{0x01, 0x31, 0x05, 0x02, 0x20, 0x21,
                   0x22, 0x23, 0x24, 0x03, 0x04};

    string res = create_message(header, body);

    print_hex(correct);
    print_hex(res);

    assert(res == correct);
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

void test_sanity_check_message_fail1() {
    string fail{0x31, 0x05, 0x02, 0x20, 0x21, 0x22, 0x23, 0x24, 0x03, 0x04};

    int res = sanity_check_message(fail);
    assert(res == -1);
}

void test_sanity_check_message_fail2() {
    string fail{0x02, 0x31, 0x05, 0x01, 0x20, 0x21,
                0x22, 0x23, 0x24, 0x03, 0x04};

    int res = sanity_check_message(fail);
    assert(res == -2);
}

void test_command_2_strings() {
    command_t cmd;
    cmd.id = 0x31;
    cmd.n_bytes = 14;
    cmd.params = {1.11, 2.22, 3.33};

    std::string correct_header{0x31, 0x0E};
    std::string correct_body(
        "\x7b\x14\x8e\x3f\x7b\x14\x0e\x40\xb8\x1e\x55\x40");

    auto [header, body] = command_2_strings(cmd);

    assert(header == correct_header);
    assert(body == correct_body);

    print_hex(correct_header);
    print_hex(header);
    print_hex(correct_body);
    print_hex(body);
}

void test_strings_2_command() {
    std::string header{0x31, 0x0E};
    std::string body("\x7b\x14\x8e\x3f\x7b\x14\x0e\x40\xb8\x1e\x55\x40");

    command_t correct_cmd;
    correct_cmd.id = 0x31;
    correct_cmd.n_bytes = 14;
    correct_cmd.params = {1.11, 2.22, 3.33};

    command_t res = strings_2_command(header, body);

    assert(res.id == correct_cmd.id);
    assert(res.n_bytes == correct_cmd.n_bytes);

    for (float f : res.params) {
        std::cout << f << std::endl;
    }

    assert(res.params.size() == correct_cmd.params.size());
    assert(res.params[0] == correct_cmd.params[0]);
    assert(res.params[1] == correct_cmd.params[1]);
    assert(res.params[2] == correct_cmd.params[2]);
}

int main() {
    test_parse_inbound_bytes();
    test_parse_outbound_bytes();

    test_create_message();

    test_retrieve_header_and_body();

    test_sanity_check_message();
    test_sanity_check_message_fail1();
    test_sanity_check_message_fail2();

    test_command_2_strings();
    test_strings_2_command();
}
