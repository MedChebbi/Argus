#include "interpolation.h"

int Interpolation :: go(int input, int duration) {
  if(input != previous){    // Interpolate to new value
    my_ramp.go(input, duration, LINEAR, ONCEFORWARD);
    previous = input;
  }
  return my_ramp.update();  // Get step
}

bool Interpolation :: finished(){
  return my_ramp.isFinished();
}
