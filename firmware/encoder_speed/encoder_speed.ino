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

int speed_max = 400;
int speed_c = -speed_max;
int dir = 1;


int TPR = 3000; // ticks per revolution
int M1_ticks = 0;
unsigned long M1_start_ticking;
int M1_window_size = 1000;

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
  // put your main code here, to run repeatedly:
  if( !md.getM1Fault() ){    
    md.setM1Speed(speed_c+=dir);
  }
  else{
    Serial.println("Fault");
  }
  if( speed_c > speed_max )
    dir = -1;
  if( speed_c < - speed_max )  
    dir = 1;
  
  Serial.print("encoder speed: ");
  Serial.println(M1_calculate_speed());
  delay(20);
}
