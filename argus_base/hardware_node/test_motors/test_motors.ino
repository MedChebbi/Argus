#include "Kinematics.h"

#define MOTOR_MAX_RPM 250        // motor's maximum rpm
#define WHEEL_DIAMETER 0.065      // robot's wheel diameter expressed in meters
#define FR_WHEEL_DISTANCE 0.6   // distance between front wheel and rear wheel
#define LR_WHEEL_DISTANCE 0.135   // distance between left wheel and right wheel
#define PWM_BITS 8              // microcontroller's PWM pin resolution.
//motor drive pins
#define l_speed_pin  5
#define l_w1         7
#define l_w2         6
#define r_w1         8
#define r_w2         9
#define r_speed_pin  11


Kinematics kinematics(MOTOR_MAX_RPM, WHEEL_DIAMETER, FR_WHEEL_DISTANCE, LR_WHEEL_DISTANCE, PWM_BITS);

double x = 0;
double y = 0;
double theta = 0;
float motor_speed = 0;

// cmd_vel variables to be received to drive with
float demandx = 0;
float demandz = 0;

int pwmL = 0;
int pwmR = 0;

// timers for the sub-main loop
unsigned long currentMillis;
long previousMillis = 0;    // set up timers
float loopTime = 10;



void setup() {

  Serial.begin(9600);
  //setup the pins for motor control
  pinMode(r_w1,OUTPUT);
  pinMode(r_w2,OUTPUT);
  pinMode(l_w1,OUTPUT);
  pinMode(l_w2,OUTPUT);
  //initialise
  sstop();
  delay(250);
}

void loop() {
  demandx = 0.3;
  demandz = -0.2;
  moving(demandx,demandz);
  delay(5);
}

void right_forward(){
  digitalWrite(r_w1,LOW);
  digitalWrite(r_w2,HIGH);
}

void left_forward(){
  digitalWrite(l_w1,LOW);
  digitalWrite(l_w2,HIGH);
}

void right_reverse(){
  digitalWrite(r_w1,HIGH);
  digitalWrite(r_w2,LOW);
}

void left_reverse(){
  digitalWrite(l_w1,HIGH);
  digitalWrite(l_w2,LOW);
}
/*
void forward(float x){
  digitalWrite(r_w1,LOW);
  digitalWrite(r_w2,HIGH);
  digitalWrite(l_w1,LOW);
  digitalWrite(l_w2,HIGH);
  x = x*100;
  x = constrain(x,0,100);
  motor_speed = map(x,0,100,0,255);

  analogWrite(r_speed_pin,int(motor_speed));
  analogWrite(l_speed_pin,int(motor_speed));
}
void backward(float x){
  digitalWrite(r_w1,HIGH);
  digitalWrite(r_w2,LOW);
  digitalWrite(l_w1,HIGH);
  digitalWrite(l_w2,LOW);
  x = x*100;
  x = constrain(x,-100,0);
  motor_speed = map(x,0,-100,0,255);

  analogWrite(r_speed_pin,int(motor_speed));
  analogWrite(l_speed_pin,int(motor_speed));
}
void rot_right(float z){
  digitalWrite(r_w1,HIGH);
  digitalWrite(r_w2,LOW);
  digitalWrite(l_w1,LOW);
  digitalWrite(l_w2,HIGH);
  z = z*100;
  z = constrain(z,-100,0);
  motor_speed = map(z,0,-100,0,255);

  analogWrite(r_speed_pin,int(motor_speed));
  analogWrite(l_speed_pin,int(motor_speed));
}
void rot_left(float z){
  digitalWrite(r_w1,LOW);
  digitalWrite(r_w2,HIGH);
  digitalWrite(l_w1,HIGH);
  digitalWrite(l_w2,LOW);
  z = z*100;
  z = constrain(z,0,100);
  motor_speed = map(z,0,100,0,255);

  analogWrite(r_speed_pin,int(motor_speed));
  analogWrite(l_speed_pin,int(motor_speed));
}*/
void sstop(){
  digitalWrite(r_w1,LOW);
  digitalWrite(r_w2,LOW);
  digitalWrite(l_w1,LOW);
  digitalWrite(l_w2,LOW);
  analogWrite(r_speed_pin,0);
  analogWrite(l_speed_pin,0);
}

void moving(float lin, float ang){
  Kinematics::output pwm;
  pwm = kinematics.getPWM(lin, 0, ang);
 if (pwm.motor1>0) {
  left_forward();
  pwmL = constrain(pwm.motor1,0,255);
  analogWrite(l_speed_pin,pwmL);
 }
 else {
  left_reverse();
  pwmL = constrain(pwm.motor1,-255,0);
  pwmL = map(pwmL,0,-255,0,255);
  analogWrite(l_speed_pin,pwmL);  
 }
 if (pwm.motor2>0){
  right_forward();
  pwmR = constrain(pwm.motor2,0,255);
  analogWrite(r_speed_pin,pwmR);
 }
 else {
  right_reverse();
  pwmR = constrain(pwm.motor2,-255,0);
  pwmR = map(pwmR,0,-255,0,255);
  analogWrite(r_speed_pin,pwmR);
 }
 /* if (lin>0) forward(lin);
  else if (lin<0) backward(lin);
  else if (ang<0) rot_right(ang);
  else if (ang>0) rot_left(ang);
  else sstop();*/
}
