#include <iomanip>
#include <iostream>
#include <vector>

enum {
    ASCII_SOH = 0x01,
    ASCII_STX = 0x02,
    ASCII_ETX = 0x03,
    ASCII_EOT = 0x04,
    ASCII_ESC = 0x1b,
    ESCAPE_OFFSET = 0x20
};

std::vector<char> TRANSMISSION_CONTROL_CHARS = {ASCII_SOH, ASCII_STX, ASCII_ETX,
                                                ASCII_EOT, ASCII_ESC};

const int BUFFER_SIZE = 2048;
char test_msg[BUFFER_SIZE] = {ASCII_SOH, 0x20, 0x21, 0x23,      ASCII_STX,
                              0x40,      0x41, 42,   ASCII_ETX, ASCII_EOT};

char test_msg2[BUFFER_SIZE] = {ASCII_ESC, 0x21,      0x23, 0x55,
                               0x66,      ASCII_ESC, 0x24};

char res[BUFFER_SIZE];
char res2[BUFFER_SIZE];

/*
 * Preface all characters found in the list TRANSMISSION_CONTROL_CHARS with the
 * escape byte ASCII_ESC and add ESCAPE_OFFSET to the byte.
 *
 * Example: \x04\x3f => \x1b\x24\x3f
 */
int parse_outbound_bytes(char *msg, char *buf) {
    int i = 0, j = 0;

    // TODO: Add guard against buffer overflow. Return error code if
    // overflowing.
    while (msg[i] != '\0') {
    loop_start:
        for (char ctrl : TRANSMISSION_CONTROL_CHARS) {
            if (msg[i] == ctrl) {
                buf[j++] = static_cast<char>(ASCII_ESC);
                buf[j++] = static_cast<char>(msg[i++] + ESCAPE_OFFSET);

                goto loop_start;
            }
        }
        buf[j++] = msg[i++];
    }
    buf[j] = '\0';

    return 0;
}

/*
 * Remove all ASCII_ESC bytes from the message buffer and remove the
 * ESCAPE_OFFSET from the next byte.
 *
 * Example: \x1b\x24\x3f => \x04\x3f
 */
int parse_inbound_bytes(char *msg, char *dest) {
    int i = 0, j = 0;

    while (msg[i] != '\0') {
        if (msg[i] == ASCII_ESC) {
            dest[j++] = static_cast<char>(msg[++i] - ESCAPE_OFFSET);
            i++;
        } else {
            dest[j++] = msg[i++];
        }
    }

    return 0;
}

void print_chars_as_hex(char *msg) {
    int i = 0;
    while (msg[i] != '\0') {
        std::cout << "\\x" << std::setw(2) << std::setfill('0') << std::hex
                  << (int)msg[i];
        i++;
    }
    std::cout << i << std::endl;
    std::cout << std::endl;
}

int main() {
    std::cout << "Original: " << std::endl;
    print_chars_as_hex(test_msg);
    parse_outbound_bytes(test_msg, res);
    std::cout << "parse_outbound_bytes: " << std::endl;
    print_chars_as_hex(res);

    std::cout << "Original: " << std::endl;
    print_chars_as_hex(test_msg2);
    parse_inbound_bytes(test_msg2, res2);
    std::cout << "parse_inbound_bytes: " << std::endl;
    print_chars_as_hex(res2);

    return 0;
}
