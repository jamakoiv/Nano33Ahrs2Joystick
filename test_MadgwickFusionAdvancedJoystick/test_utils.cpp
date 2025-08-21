#include "../MadgwickFusionAdvancedJoystick/src/utils.h"
#include <cassert>

void test_set_calib_helper_vector() {
  std::vector<float> orig;
  orig.push_back(1.11);
  orig.push_back(2.22);
  orig.push_back(3.33);

  FusionVector correct = {1.11, 2.22, 3.33};
  FusionVector res;

  int res_code = set_calib_helper(orig, res);

  assert(res.axis.x == correct.axis.x);
  assert(res.axis.y == correct.axis.y);
  assert(res.axis.z == correct.axis.z);
  assert(res_code == 0);
}

void test_set_calib_helper_vector_fail() {
  std::vector<float> orig;
  orig.push_back(1.11);
  orig.push_back(2.22);

  FusionVector res;
  int res_code = set_calib_helper(orig, res);
  assert(res_code == -1);
}

void test_set_calib_helper_vector_vector() {
  std::vector<float> orig;
  orig.push_back(1.11);
  orig.push_back(2.22);
  orig.push_back(3.33);
  orig.push_back(4.44);
  orig.push_back(5.55);
  orig.push_back(6.66);

  FusionVector correctA = {1.11, 2.22, 3.33};
  FusionVector correctB = {4.44, 5.55, 6.66};
  FusionVector resA, resB;

  int res_code = set_calib_helper(orig, resA, resB);

  assert(resA.axis.x == correctA.axis.x);
  assert(resA.axis.y == correctA.axis.y);
  assert(resA.axis.z == correctA.axis.z);

  assert(resB.axis.x == correctB.axis.x);
  assert(resB.axis.y == correctB.axis.y);
  assert(resB.axis.z == correctB.axis.z);

  assert(res_code == 0);
}

void test_set_calib_helper_vector_vector_fail() {
  std::vector<float> orig;
  orig.push_back(1.11);
  orig.push_back(2.22);
  orig.push_back(3.33);
  orig.push_back(4.44);
  orig.push_back(5.55);

  FusionVector resA, resB;
  int res_code = set_calib_helper(orig, resA, resB);

  assert(res_code == -1);
}

void test_set_calib_helper_vector_matrix() {
  std::vector<float> orig;
  orig.push_back(1.11);
  orig.push_back(2.22);
  orig.push_back(3.33);
  orig.push_back(4.44);
  orig.push_back(5.55);
  orig.push_back(6.66);

  FusionVector correctA = {1.11, 2.22, 3.33};
  FusionMatrix correctB;
  correctB.element.xx = 1 / 4.44;
  correctB.element.yy = 1 / 5.55;
  correctB.element.zz = 1 / 6.66;

  FusionVector resA;
  FusionMatrix resB;
  int res_code = set_calib_helper(orig, resA, resB);

  assert(resA.axis.x == correctA.axis.x);
  assert(resA.axis.y == correctA.axis.y);
  assert(resA.axis.z == correctA.axis.z);

  assert(resB.element.xx == correctB.element.xx);
  assert(resB.element.yy == correctB.element.yy);
  assert(resB.element.zz == correctB.element.zz);

  assert(res_code == 0);
}
void test_set_calib_helper_vector_matrix_fail() {}
