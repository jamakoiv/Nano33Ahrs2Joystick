#include "test_serial_utils.h"
#include <cassert>

void test_parse_outbound_bytes() {
    string test_msg{0x01, 'a', 'b', 0x02, 'c', 'd', 0x03, 'e', 'f', 0x04};

    string correct{0x1b, 0x21, 'a',  'b', 0x1b, 0x22, 'c',
                   'd',  0x1b, 0x23, 'e', 'f',  0x1b, 0x24};

    print_hex(test_msg);
    print_hex(correct);

    string res;
    parse_outbound_bytes(test_msg, res);
    print_hex(res);
    assert(correct == res);
}

void test_parse_inbound_bytes() {
    string test_msg{0x1b, 0x21, 'a',  'b', 0x1b, 0x22, 'c',
                    'd',  0x1b, 0x23, 'e', 'f',  0x1b, 0x24};

    string correct{0x01, 'a', 'b', 0x02, 'c', 'd', 0x03, 'e', 'f', 0x04};

    print_hex(test_msg);
    print_hex(correct);

    string res;
    parse_inbound_bytes(test_msg, res);
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

void test_sanity_check_message() {}

int main() {
    test_parse_inbound_bytes();
    test_parse_outbound_bytes();
    test_create_message();
    test_sanity_check_message();
}
