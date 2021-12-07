#include "trigonometry.h"

extern float wrist_angle;

static uint8_t parallel = 0;

// Calculates angles for the 3 joints in radians 
float* get_angles(float x, float z){
  static float angles[NUM_ANGLES];
  float d, beta, gamma, safe_beta; // Intermediary variables

  // Illustration: https://github.com/MedChebbi/Argus/blob/master/robotic_arm/images/trigonometry.png
  d = sqrt(x*x + z*z);
  // The value was getting out of acos' range (acos [-1, 1]) so capped it to [-1, 1]
  safe_beta = abs(d/(2*SEGMENT_LEN)) > 1 ? sign(d/(2*SEGMENT_LEN)) : d/(2*SEGMENT_LEN);
  beta = acos(safe_beta);
  gamma = atan(safe_div(x, z));

  // Shoulder, elbow, wrist (Radians --> degrees)
  angles[0] = (beta + gamma) * (180/PI);
  angles[1] = (PI - 2 * beta) * (180/PI);

  if(parallel) angles[2] = (PI + beta - gamma) * (180/PI); // Always parallel to ground --> bad for gripping from ground
  else angles[2] = wrist_angle;

  return angles;
} 

// a/b with appropriate handling of division by 0 issue
static float safe_div(int a, int b){
  // b == 0 <==> a/b --> "infinity"
  return b != 0 ? a/b : 1e8;
}

static int8_t sign(float x){
  return x / abs(x);
}

void set_parallel(){
  parallel = 1;
}

void reset_parallel(){
  parallel = 0;
}
