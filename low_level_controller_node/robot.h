/*
 * Single file class for robot odometry (twist and pose keeping).
 */

#ifndef __ROBOT_H__
  #define __ROBOT_H__

  #include "ARGUS_configs.h"

  /* ---------------------------------------------- */
  /* ----------------- CLASS BODY ----------------- */
  /* ---------------------------------------------- */
  
  class robot{
    private:

      // These are the info about the robot --> Pose and Twist
      float x, y, theta;
      float linear_x, linear_y, omega;
      unsigned long last_t;
      
    public:

      // Update the info about the robot
      void update(const unsigned long present_time, float dr, float dl);
      void reset();
      void reset(float x_, float y_, float theta_);

      // Get the info about the robot
      void get_pose(float &x, float &y, float &th);
      void get_twist(float &lin_x, float &lin_y, float &d_theta);
  };

  /* --------------------------------------------------- */
  /* ----------------- CLASS FUNCTIONS ----------------- */
  /* --------------------------------------------------- */

  // Input : time, distance_traveled_by_right_wheel, distance_traveled_by_left_wheel
  void robot :: update(const unsigned long present_time, float dr, float dl){
    unsigned long dt;
    float center_dist, d_th;

    // Ref: https://ocw.mit.edu/courses/electrical-engineering-and-computer-science/6-186-mobile-autonomous-systems-laboratory-january-iap-2005/study-materials/odomtutorial.pdf
    center_dist = (dr + dl) / 2;
    x += center_dist * cos(theta);
    x += center_dist * sin(theta);

    theta += (d_th = (dr - dl) / BASELINE_DISTANCE);
    
    dt = present_time - last_t;
    
    if(dt == 0) linear_x = linear_y = omega = 0;
    else{
      linear_x = center_dist / dt;
      linear_y = 0;
      omega = d_th / dt; 
    }
    last_t = present_time;
  }

  // Reset the robot position and state to absolute 0
  void robot :: reset(){ x = y = theta = linear_x = linear_y = omega = 0, last_t = micros(); }

  // Reset the robot position and state to given position
  void robot :: reset(float x_, float y_, float theta_){ x = x_, y = y_, theta = theta_, last_t = micros(); }

  void robot :: get_pose(float &x_, float &y_, float &th){ x_ = x, y_ = y, th = theta; }
  
  void robot :: get_twist(float &lin_x, float &lin_y, float &d_theta){ lin_x = linear_x, lin_y = linear_y, d_theta = omega; }
  
#endif // __ROBOT_H__
