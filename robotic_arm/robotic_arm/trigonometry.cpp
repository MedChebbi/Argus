#include "trigonometry.h"

// Calculates angles for the 3 joints in radians
float* get_angles(float x, float z){
  static float angles[NUM_ANGLES];
  float d, beta, gamma; // Intermediary variables

  d = sqrt(x*x + z*z);  // Distance from center of the plane to target point
  beta = acos(d/(2*SEGMENT_LEN));
  gamma = atan(x/z);

  // Shoulder, elbow, wrist
  angles[0] = beta + gamma;
  angles[1] = PI - 2 * beta;
  // angles[2] = ??? // Dunno either fix it or make it mobile
  angles[2] = PI + beta - gamma; // Always parallel to ground --> bad for gripping from ground

  return angles;
}

// Radians --> degrees --> milliseconds (deg° ~ 11ms) 
int rad_to_ms(float ang){
  return ang * (180/PI) * DEG_TO_MS ;
}
