#include "test_utils.h"

int main() {
  test_set_calib_helper_vector();
  test_set_calib_helper_vector_fail();

  test_set_calib_helper_vector_vector();
  test_set_calib_helper_vector_vector_fail();

  test_set_calib_helper_vector_matrix();
  test_set_calib_helper_vector_matrix_fail();

  return 0;
}
