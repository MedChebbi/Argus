#include "trigonometry.h"

// Calculates angles for the 3 joints in radians
float* get_angles(float x, float z){
  static float angles[NUM_ANGLES];
  float d, beta, gamma; // Intermediary variables

  d = sqrt(x*x + z*z);  // Distance from center of the plane to target point
  beta = acos(d/(2*SEGMENT_LEN));
  gamma = atan(x/z);

  // Shoulder, elbow, wrist (Radians --> degrees)
  angles[0] = (beta + gamma) * (180/PI);
  angles[1] = (PI - 2 * beta) * (180/PI);
  // angles[2] = ??? // Dunno either fix it or make it mobile
  angles[2] = (PI + beta - gamma) * (180/PI); // Always parallel to ground --> bad for gripping from ground

  return angles;
} 
