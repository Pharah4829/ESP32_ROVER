#include <Kaeri_MotorDriver.h>

#define Motor1A 1
#define Motor1B 2
#define Encoder1A 3
#define Encoder1B 4
#define Motor2A 6
#define Motor2B 7
#define Encoder2A 8
#define Encoder2B 9

void setup(){
  Serial.begin(115200);
  Driver.MotorSetup(Motor1A, Motor1B, Motor2A, Motor2B);
  Driver.EncoderSetup(Encoder1A, Encoder1B, Encoder2A, Encoder2B);
  Driver.PIDset(5.0f, 2.0f, 0.0f);
  Driver.Begin(1);
}

void loop(){
  int speed=150 * sin(PI * (millis() / 1000.0));
  Driver.MotorSetSpeed(speed,-speed);
}
