#include <PID_v1.h>
#include "DualVNH5019MotorShield.h"
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

#define M1INA 2
#define M1INB 4
#define M1ENDIAG 6
#define M2INA 7
#define M2INB 8
#define M1PWM 9
#define M2PWM 10
#define M2ENDIAG 12
#define M1CS A0
#define M2CS A1

#define M1EN1 3
#define M1EN2 5

#define M2EN1 3
#define M2EN2 5

/* MOTORS */
DualVNH5019MotorShield md(M1INA, M1INB, M1PWM, M1ENDIAG, M1CS, M2INA, M2INB, M2PWM, M2ENDIAG, M2CS);
unsigned long last_cmd_time = 0;
float stop_motor_time_s = 2;

/* ENCODER STUFF */
int TPR = 3000; // ticks per revolution
int M1_ticks = 0, M2_ticks = 0;
unsigned long M1_start_ticking, M2_start_ticking;
int ticks_window_size = 1000;

void m1_encoder_cb(){
  digitalRead(M1EN1) != digitalRead(M1EN2) ? M1_ticks++ : M1_ticks--;
}

void m2_encoder_cb(){
  digitalRead(M2EN1) != digitalRead(M2EN2) ? M2_ticks++ : M2_ticks--;
}

/* PID STUFF*/
// stores for PID
double input_speed[2];
double output_speed[2];
double target_speed[2];
PID* M1PID;
PID* M2PID;

float kP = 10, kD = 0, kI = 0;

/* ROS STUFF */
ros::NodeHandle  nh;
// speed M1, rad/s
// speed M2, rad/s
// current M1, amps
// current M2, amps
std_msgs::Float32MultiArray motor_info_msg;

void target_speed_cb(const std_msgs::Float32MultiArray& msg){
  target_speed[0] = double(msg.data[0]);
  target_speed[1] = double(msg.data[1]);
  last_cmd_time = millis();
}

ros::Publisher motor_info_pub("~motor_info", &motor_info_msg);
ros::Subscriber<std_msgs::Float32MultiArray> target_speed_sub("motor_speed", &target_speed_cb);

/* REAL */

void setup() {  
  // ROS
  nh.initNode();
  nh.advertise(motor_info_pub);  
  nh.subscribe(target_speed_sub);

  while (!nh.connected()) {
      nh.spinOnce();
  }
  motor_info_msg.data = (float*)malloc(sizeof(float) * 4);
  motor_info_msg.data_length = 4;

  // TODO PARAMS

  // motor
  md.init();    
  // encoder
  attachInterrupt( digitalPinToInterrupt(M1EN1), m1_encoder_cb, CHANGE);
  attachInterrupt( digitalPinToInterrupt(M2EN1), m2_encoder_cb, CHANGE);
  // PIDs
  M1PID = new PID(input_speed, output_speed, target_speed, kP, kI, kD, DIRECT);
  M1PID->SetMode(AUTOMATIC);
  M1PID->SetOutputLimits(-400, 400);
  M1PID->SetSampleTime(200);
  M2PID = new PID(input_speed+1, output_speed+1, target_speed+1, kP, kI, kD, DIRECT);
  M2PID->SetMode(AUTOMATIC);
  M2PID->SetOutputLimits(-400, 400);
  M2PID->SetSampleTime(200);
  
}

float M1_calculate_speed(){
  unsigned long dt = millis() - M1_start_ticking;  
  if( abs(M1_ticks) > ticks_window_size ){
    M1_ticks = 0;
    M1_start_ticking = millis();  
  }  
  float speed_rad_per_sec = 2 * 3.1415 * 1000 * float(M1_ticks) / (dt * TPR);
  return speed_rad_per_sec;
}

float M2_calculate_speed(){
  unsigned long dt = millis() - M2_start_ticking;  
  if( abs(M2_ticks) > ticks_window_size ){
    M2_ticks = 0;
    M2_start_ticking = millis();  
  }  
  float speed_rad_per_sec = 2 * 3.1415 * 1000 * float(M2_ticks) / (dt * TPR);
  return speed_rad_per_sec;
}

void sendData(){
  motor_info_msg.data[0] = input_speed[0];
  motor_info_msg.data[1] = input_speed[1];
  motor_info_msg.data[2] = float(md.getM1CurrentMilliamps()) / 1000;
  motor_info_msg.data[3] = float(md.getM2CurrentMilliamps()) / 1000;
  
  motor_info_pub.publish(&motor_info_msg);
}

void loop() {
  M1_calculate_speed();
  M2_calculate_speed();
  /*
  if( millis() - last_cmd_time > 1000 * stop_motor_time_s ){
    target_speed[0] = 0;
    target_speed[1] = 0;
    md.setSpeeds(0, 0);          
  }
  else{    
  */
    M1PID->Compute();
    M2PID->Compute();
    md.setSpeeds(output_speed[0], output_speed[1]);
  //}
  sendData();
  nh.spinOnce();
  delay(20);
}
