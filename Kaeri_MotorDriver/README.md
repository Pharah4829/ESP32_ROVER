# Kaeri_MotorDriver

ESP32(Arduino 코어)에서 **5ms 백그라운드 PID**로 2개 모터를 제어하는 드라이버 라이브러리.

- 하드웨어 타이머 ISR → FreeRTOS 태스크 알림 → **PID 루프는 태스크에서 실행**
- PWM은 **LEDC**를 사용(주기 고정, 듀티만 갱신)
- 외부 API 4개만 사용:
  - `Driver.MotorSetup(m1A, m1B, m2A, m2B)`
  - `Driver.EncoderSetup(e1A, e1B, e2A, e2B)`
  - `Driver.Begin(period_ms=5)`
  - `Driver.PIDset(kp, ki, kd)`
  - `Driver.MotorSetSpeed(m1_rpm, m2_rpm)`

## 설치
- 이 폴더를 통째로 ZIP → Arduino IDE에서 **스케치 > 라이브러리 포함하기 > .ZIP 라이브러리 추가**
- 혹은 `Documents/Arduino/libraries/`에 `Kaeri_MotorDriver` 폴더로 복사

## 주의
- `TICKS_PER_REV`(엔코더 1회전 카운트) 값을 소스(`Kaeri_MotorDriver.cpp`)에서 **하드웨어에 맞게 조정**하세요.
- PID는 기본 5ms 고정 샘플링(`DT_FIXED_S`)으로 계산합니다.

