#ifndef __KINEMATICS_CONTROLLER_H__
  #define __KINEMATICS_CONTROLLER_H__

  #include "ARGUS_configs.h"

  typedef struct{
    float left_wheel_rpm;
    float right_wheel_rpm;
  }RPM_RL;

  RPM_RL get_rpm_from_vel(float linear_x, float angular_z);

  /* ---------------------------------------------- */
  /* ----------- FUNCTIONS' DECLARATION ----------- */
  /* ---------------------------------------------- */
  
  RPM_RL get_rpm_from_vel(float linear_x, float angular_z){
    return {
        /* left wheel rpm */  (linear_x - angular_z * 0.5f * BASELINE_DISTANCE)/(RPM_TO_RAD_PER_S * DIST_PER_RADIAN),
        /* right wheel rpm */ (linear_x + angular_z * 0.5f * BASELINE_DISTANCE)/(RPM_TO_RAD_PER_S * DIST_PER_RADIAN),
    };
  }
  
#endif // __KINEMATICS_CONTROLLER_H__
