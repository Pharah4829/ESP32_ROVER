#pragma once
#include <Arduino.h>

// 사용 예:
//   Driver.MotorSetup(M1A, M1B, M2A, M2B);
//   Driver.EncoderSetup(E1A, E1B, E2A, E2B);
//   Driver.PIDset(5.0f, 10.0f, 0.0f);
//   Driver.Begin(5);                  // 5ms 주기
//   Driver.MotorSetSpeed(50, -50);    // 목표속도(RPM 가정)

class KaeriMotorDriver {
public:
  // 1) 모터 핀 설정
  void MotorSetup(int m1A, int m1B, int m2A, int m2B);

  // 1) 엔코더 핀 설정
  void EncoderSetup(int enc1A, int enc1B, int enc2A, int enc2B);

  // 2) 셋업 + 타이머/태스크 시작 (period_ms: 기본 5ms)
  //    true 반환 시 정상 시작
  bool Begin(uint32_t period_ms = 5);

  // 3) PID 계수 설정
  void PIDset(float kp, float ki, float kd);

  // 4) 목표 속도 설정 (단위: RPM 가정, 필요시 코드 내 TICKS_PER_REV 조정)
  void MotorSetSpeed(int m1_rpm, int m2_rpm);

private:
  // 핀
  int _m1A=-1, _m1B=-1, _m2A=-1, _m2B=-1;
  int _e1A=-1, _e1B=-1, _e2A=-1, _e2B=-1;

  // LEDC 채널/설정
  int _ch_m1A=0, _ch_m1B=1, _ch_m2A=2, _ch_m2B=3;
  uint32_t _pwm_freq = 20000;   // 20 kHz
  uint8_t  _pwm_bits = 8;       // 0~255 duty

  // 타이머/태스크
  hw_timer_t*  _timer = nullptr;
  TaskHandle_t _task  = nullptr;
  uint32_t     _period_us = 5000;

  // 엔코더 카운트(ISR에서만 갱신)
  volatile long _m1pos_i = 0, _m2pos_i = 0;

  // 목표 속도(RPM)
  volatile int _m1Target = 0, _m2Target = 0;

  // PID 계수
  volatile float _kp=0, _ki=0, _kd=0;

  // 내부 상태
  long  _m1pos=0, _m2pos=0, _m1posPrev=0, _m2posPrev=0;
  float _m1rpmprev=0, _m2rpmprev=0;
  float _m1rpmf=0,   _m2rpmf=0;
  float _m1int=0,    _m2int=0;
  float _m1lerr=0,   _m2lerr=0;

  // 아주 짧게만 쓰는 임계구역
  portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;

  // 내부 구현
  void _setupLEDC();
  void _writeMotor(int chA, int chB, float u); // u: -255~+255
  void _pidStep();                             // 5ms마다 실행

  // 정적 트램펄린/ISR
  static void IRAM_ATTR _timerISR_tramp();
  static void IRAM_ATTR _enc1ISR();
  static void IRAM_ATTR _enc2ISR();
  static void _taskEntry(void* arg);

  // 트램펄린용 싱글턴 포인터
  static KaeriMotorDriver* _self;
};

// 전역 인스턴스 (요구한 대로 Driver 명)
extern KaeriMotorDriver Driver;
