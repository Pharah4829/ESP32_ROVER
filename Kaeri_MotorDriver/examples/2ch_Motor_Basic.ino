#include <Kaeri_MotorDriver.h>

#define Motor1A   1
#define Motor1B   2
#define Encoder1A 3
#define Encoder1B 4
#define Motor2A   6
#define Motor2B   7
#define Encoder2A 8
#define Encoder2B 9

void setup() {
  Serial.begin(115200);

  Driver.MotorSetup(Motor1A, Motor1B, Motor2A, Motor2B);
  Driver.EncoderSetup(Encoder1A, Encoder1B, Encoder2A, Encoder2B);
  Driver.PIDset(5.0f, 10.0f, 0.0f);

  if (!Driver.Begin(5)) {   // 5ms 주기
    Serial.println("Begin failed (check pins)");
    while(1);
  }

  // PID는 이제 백그라운드에서 계속 동작
  Driver.MotorSetSpeed(0, 0);
}

void loop() {
  Driver.MotorSetSpeed(50, -50);  // RPM 가정
  delay(2000);
  Driver.MotorSetSpeed(0, 0);
  delay(2000);
}
