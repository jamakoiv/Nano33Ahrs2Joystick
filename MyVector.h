
#ifndef _MY_VECTOR_
  #define _MY_VECTOR_

#include <Arduino.h>
#include <math.h>
#include <string>

namespace MyVector {

const uint8_t PRINT_DECIMAL_PLACES = 3;

//////////////////////////////////////////////////////////////////////////////////////////
// Simple class for dealing with basic vector math for 3-dimensional vector in Arduino. //
//////////////////////////////////////////////////////////////////////////////////////////
class vector {

public:
  float x{0}, y{0}, z{0}; 

  // Constructors
  vector();
  vector(float x, float y, float z); // Init with int, float etc should be handled automatically.

  // Destructor
  ~vector();

  /////////////
  // METHODS // 
  /////////////

  double norm(void) const;        // Calculate vector norm / magnitude / total strength.
  void normalize(void);           // Normalize vector magnitude to 1.

  std::string to_string(void) const;
  void printVector(void) const;     // Print vector as (x,y,z) to Serial.  
  void printVectorXYZ(void) const;  // Print x,y,z values to Serial. 

  static vector crossProduct(const vector &A, const vector &B); // Calculate the cross product AxB.
  static double dotProduct(const vector &A, const vector &B);   // Calculate the dot-product A'B.

  static double vectorAngle(const vector &A, const vector &B);  // Calculate the angle of vectors Aand B. 
                                                                // Result in radians.

  ///////////////
  // Operators //
  ///////////////

  // TODO: Do we need 'vector + Constant' and 'vector - Constant'?
  // TODO: Maybe assign dot- or cross-product to 'vector * vector'?

  vector operator + (const vector &A);    // Enable 'vector = vector + vector'.
  vector operator - (const vector &A);    // Enable 'vector = vector - vector'.
  vector operator * (const vector &C);    // Enable 'vector = vector * vector'.
  vector operator * (const float C);      // Enable 'vector = vector * Constant'.
  vector operator / (const vector &A);    // Enable 'vector = vector / vector'.
  vector operator / (const float C);      // Enable 'vector = vector / Constant'. 
  vector& operator -= (const vector &A);  // Enable 'vector -= vector'.
  vector& operator += (const vector &A);  // Enable 'vector += vector'.
  vector& operator *= (const float C);    // Enable 'vector *= Constant'.
  vector& operator /= (const float C);    // Enable 'vector /= Constant'. 


} ; // End of class 'vector'.

} // End of namespace MyVector.

#endif

