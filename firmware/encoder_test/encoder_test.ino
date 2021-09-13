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

int m1_encoder_cntr = 0;
int start_speed = -400;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  md.init();
  attachInterrupt( digitalPinToInterrupt(M1EN1), m1_encoder_cb, CHANGE);
}

void m1_encoder_cb(){
  if ( digitalRead(M1EN2) != digitalRead(M1EN1) )
    m1_encoder_cntr++;
  else
    m1_encoder_cntr--;
}


void loop() {
  // put your main code here, to run repeatedly:
  if( !md.getM1Fault() ){
    //md.setM1Speed(start_speed++);
    md.setM1Speed(10);
  }
  else{
    Serial.println("Fault");
  }

  Serial.print("M1 speed:");
  Serial.print(start_speed);
  Serial.print("encoders: ");
  Serial.println(m1_encoder_cntr);
  delay(20);
}
