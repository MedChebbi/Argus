#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>

ros::NodeHandle nh;

//motor drive pins
const int l_speed_pin = 5;
const int l_w1 = 7;
const int l_w2 = 6;
const int r_w1 = 9;
const int r_w2 = 8;
const int r_speed_pin = 10;

double x = 0;
double y = 0;
double theta = 0;
float motor_speed =0;

// cmd_vel variables to be received to drive with
float demandx;
float demandz;

// timers for the sub-main loop
unsigned long currentMillis;
long previousMillis = 0;    // set up timers
float loopTime = 10;

void velCallback(  const geometry_msgs::Twist& vel)
{
     demandx = vel.linear.x;
     demandz = vel.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> sub("control/cmd_vel" , velCallback);

void setup() {
  nh.getHardware()->setBaud(115200);      // set baud rate to 115200
  nh.initNode();              // init ROS
  nh.subscribe(sub);          // subscribe to cmd_vel

  //setup the pins for motor control
  pinMode(r_w1,OUTPUT);
  pinMode(r_w2,OUTPUT);
  pinMode(l_w1,OUTPUT);
  pinMode(l_w2,OUTPUT);
  //initialise
  sstop();
  delay(500);
}

void loop() {
  nh.spinOnce();
  moving(demandx,demandz);
  delay(5);
}

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
}
void sstop(){
  digitalWrite(r_w1,LOW);
  digitalWrite(r_w2,LOW);
  digitalWrite(l_w1,LOW);
  digitalWrite(l_w2,LOW);
  analogWrite(r_speed_pin,0);
  analogWrite(l_speed_pin,0);
}

void moving(float lin, float ang){
  if (lin>0) forward(lin);
  else if (lin<0) backward(lin);
  else if (ang<0) rot_right(ang);
  else if (ang>0) rot_left(ang);
  else sstop();
}
