#include <PID_v1.h>

#include "DualVNH5019MotorShield.h"

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

DualVNH5019MotorShield md(M1INA, M1INB, M1PWM, M1ENDIAG, M1CS, M2INA, M2INB, M2PWM, M2ENDIAG, M2CS);

double speed_max = 1;
double speed_c = -speed_max;
double dir = 0.01;

double input_speed;
double output_speed;
double target_speed;

int TPR = 3000; // ticks per revolution
int M1_ticks = 0;
unsigned long M1_start_ticking;
int M1_window_size = 1000;

PID *M1PID;

void m1_encoder_cb(){
  if( digitalRead(M1EN1) != digitalRead(M1EN2) )
    M1_ticks++;
  else
    M1_ticks--;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  md.init();

  M1_start_ticking = millis();
  attachInterrupt( digitalPinToInterrupt(M1EN1), m1_encoder_cb, CHANGE);    

  M1PID = new PID(&input_speed, &output_speed, &target_speed, 100, 0, 0, DIRECT);
  M1PID->SetMode(AUTOMATIC);
  M1PID->SetOutputLimits(-400,400);
  M1PID->SetSampleTime(200);
 
}


float M1_calculate_speed(){
  unsigned long dt = millis() - M1_start_ticking;  
  if( abs(M1_ticks) > M1_window_size ){
    M1_ticks = 0;
    M1_start_ticking = millis();  
  }  
  float speed_ticks_per_millis = 2 * 3.1415 * 1000 * float(M1_ticks) / (dt * TPR);
  return speed_ticks_per_millis;
}


void loop() {
  target_speed = speed_c+=dir;
  input_speed = M1_calculate_speed();
    
  M1PID->Compute();
    
  if( !md.getM1Fault() ){    
    md.setM1Speed(output_speed);
  }
  else{
    Serial.println("Fault");
  }
  if( speed_c > speed_max )
    dir = -dir;
  if( speed_c < - speed_max )  
    dir = -dir;
  
  Serial.print("encoder speed: ");
  Serial.print(input_speed);
  Serial.print(" ");
  Serial.print(target_speed);
  Serial.print(" ");
  Serial.println(output_speed);
  
  delay(20);
}
