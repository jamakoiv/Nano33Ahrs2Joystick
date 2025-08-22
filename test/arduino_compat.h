#include <iostream>

enum { BIN = -10, OCT = -11, DEC = -12, HEX = -13 };

class Serial {

  void println(char *msg);
  void println(int n, int format);
  void println(float n, int format);
};
