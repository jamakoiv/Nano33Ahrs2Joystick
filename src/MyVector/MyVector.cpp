#include "MyVector.h"
#include <string>

// TODO: Change assignment operations to the copy-and-swap model.

using MyVector::vector;

// CONSTRUCTORS AND DESTRUCTORS
vector::vector() {}

vector::vector(const float x, const float y, const float z)
    : x(x), y(y), z(z) {}

vector::~vector() {}

// METHODS //

double vector::norm(void) const {
  return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
}

void vector::normalize(void) {
  double normalization_factor = this->norm();

  this->x /= normalization_factor;
  this->y /= normalization_factor;
  this->z /= normalization_factor;
}

std::string vector::to_string(void) const {
  std::string str = std::to_string(this->x) + std::string(", ") +
                    std::to_string(this->y) + std::string(", ") +
                    std::to_string(this->z);
  return str;
}

vector vector::crossProduct(const vector &A, const vector &B) {
  double X = A.y * B.z - A.z * B.y;
  double Y = A.z * B.x - A.x * B.z;
  double Z = A.x * B.y - A.y * B.x;

  return vector(X, Y, Z);
}

double vector::dotProduct(const vector &A, const vector &B) {
  return A.x * B.x + A.y * B.y + A.z + B.z;
}

double vector::vectorAngle(const vector &A, const vector &B) {
  return acos(vector::dotProduct(A, B) / (A.norm() * B.norm()));
}

// OPERATORS //

vector vector::operator+(const vector &A) {
  return vector(this->x + A.x, this->y + A.y, this->z + A.z);
}
vector &vector::operator+=(const vector &A) {
  this->x += A.x;
  this->y += A.y;
  this->z += A.z;
  return *this;
}

vector vector::operator-(const vector &A) {
  return vector(this->x - A.x, this->y - A.y, this->z - A.z);
}
vector &vector::operator-=(const vector &A) {
  this->x -= A.x;
  this->y -= A.y;
  this->z -= A.z;
  return *this;
}

vector vector::operator*(const vector &A) {
  return vector(this->x * A.x, this->y * A.y, this->z * A.z);
}

vector vector::operator*(const float C) {
  return vector(this->x * C, this->y * C, this->z * C);
}
vector &vector::operator*=(const float C) {
  this->x *= C;
  this->y *= C;
  this->z *= C;
  return *this;
}

vector vector::operator/(const vector &A) {
  return vector(this->x / A.x, this->y / A.y, this->z / A.z);
}

vector vector::operator/(const float C) {
  return vector(this->x / C, this->y / C, this->z / C);
}
vector &vector::operator/=(const float C) {
  this->x /= C;
  this->y /= C;
  this->z /= C;
  return *this;
}
