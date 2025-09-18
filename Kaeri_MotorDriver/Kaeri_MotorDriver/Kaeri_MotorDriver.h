#pragma once
#include <Arduino.h>

class KaeriMotorDriver {
public:
  // 1) 모터/엔코더 핀 설정
  void MotorSetup(int m1A, int m1B, int m2A, int m2B);
  void EncoderSetup(int e1A, int e1B, int e2A, int e2B);

  // 2) FreeRTOS PID 시작: period_ms(기본 2ms)
  bool Begin(uint32_t period_ms = 2);

  // 3) PID 계수 설정
  void PIDset(float kp, float ki, float kd);

  // 4) 목표 속도(RPM) 설정
  void MotorSetSpeed(int m1_rpm, int m2_rpm);

private:
  // 핀
  int _m1A=-1, _m1B=-1, _m2A=-1, _m2B=-1;
  int _e1A=-1, _e1B=-1, _e2A=-1, _e2B=-1;

  // 목표 속도(RPM)
  volatile int _m1Target = 0, _m2Target = 0;

  // 엔코더 카운트(ISR)
  volatile long _m1pos_i = 0, _m2pos_i = 0;

  // 내부 상태
  long  _m1posPrev=0, _m2posPrev=0;
  float _m1rpmf=0, _m2rpmf=0;
  float _m1rpmprev=0, _m2rpmprev=0;
  float _m1int=0, _m2int=0;
  float _m1lerr=0, _m2lerr=0;

  // PID 계수
  volatile float _kp=5.0f, _ki=10.0f, _kd=0.0f;

  // FreeRTOS
  TaskHandle_t _task = nullptr;
  TickType_t   _period_ticks = 0;

  // 타이밍/상수
  static constexpr float TICKS_PER_REV = 600.0f; // 하드웨어에 맞게
  float _dt_s = 0.002f;                          // Begin에서 갱신됨 (초)

  // 필터 계수(그대로 사용)
  static constexpr float RPM_LPF_A = 0.854f;
  static constexpr float RPM_LPF_B = 0.0728f;

  // 내부 구현
  void _pidStep();

  // ISR/Task 정적 트램펄린
  static void IRAM_ATTR _enc1ISR();
  static void IRAM_ATTR _enc2ISR();
  static void _taskEntry(void* arg);

  // 싱글턴 포인터
  static KaeriMotorDriver* _self;
};

extern KaeriMotorDriver Driver;
