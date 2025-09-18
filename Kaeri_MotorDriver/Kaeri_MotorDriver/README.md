# Kaeri_MotorDriver
- ESP32만으로 2채널 DC 모터를 엔코더 피드백 + PID로 제어하는 초경량 라이브러리.
- 소프트웨어 카운터로 엔코더를 측정, FreeRTOS 태스크에서 지정 주기마다 PID Loop 를 수행.
- 외부 드라이버 라이브러리 없이 Arduino 코어만 사용해 제어 수행.

# API
- 핀 셋업
  - `Driver.MotorSetup(m1A, m1B, m2A, m2B)`
  - `Driver.EncoderSetup(e1A, e1B, e2A, e2B)`
    
- period_ms 로 PID Loop 시작
  - `Driver.Begin(period_ms)`

- PID 계수 튜닝
  - `Driver.PIDset(kp, ki, kd)`
 
- 모터 속도재어
  - `Driver.MotorSetSpeed(m1_rpm, m2_rpm)`

## 설치
- 이 폴더를 통째로 ZIP → Arduino IDE에서 **스케치 > 라이브러리 포함하기 > .ZIP 라이브러리 추가**
- 혹은 `Documents/Arduino/libraries/`에 `Kaeri_MotorDriver` 폴더로 복사

