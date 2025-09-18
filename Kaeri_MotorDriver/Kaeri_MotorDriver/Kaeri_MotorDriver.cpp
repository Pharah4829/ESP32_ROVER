#include "Kaeri_MotorDriver.h"

KaeriMotorDriver* KaeriMotorDriver::_self = nullptr;
KaeriMotorDriver Driver;

// --- 엔코더 ISR ---
void IRAM_ATTR KaeriMotorDriver::_enc1ISR(){
  int b = digitalRead(Driver._self->_e1B);
  if (b>0) Driver._self->_m1pos_i--;
  else     Driver._self->_m1pos_i++;
}
void IRAM_ATTR KaeriMotorDriver::_enc2ISR(){
  int b = digitalRead(Driver._self->_e2B);
  if (b>0) Driver._self->_m2pos_i--;
  else     Driver._self->_m2pos_i++;
}

// --- PID 태스크 엔트리 ---
void KaeriMotorDriver::_taskEntry(void* arg){
  auto* self = static_cast<KaeriMotorDriver*>(arg);
  TickType_t last = xTaskGetTickCount();
  for(;;){
    self->_pidStep();
    vTaskDelayUntil(&last, self->_period_ticks); // Begin에서 설정한 주기
  }
}

// --- API 구현 ---
void KaeriMotorDriver::MotorSetup(int m1A, int m1B, int m2A, int m2B){
  _m1A=m1A; _m1B=m1B; _m2A=m2A; _m2B=m2B;
  pinMode(_m1A, OUTPUT);
  pinMode(_m1B, OUTPUT);
  pinMode(_m2A, OUTPUT);
  pinMode(_m2B, OUTPUT);
}

void KaeriMotorDriver::EncoderSetup(int e1A, int e1B, int e2A, int e2B){
  _e1A=e1A; _e1B=e1B; _e2A=e2A; _e2B=e2B;
  pinMode(_e1A, INPUT);
  pinMode(_e1B, INPUT);
  pinMode(_e2A, INPUT);
  pinMode(_e2B, INPUT);
}

bool KaeriMotorDriver::Begin(uint32_t period_ms){
  if (_m1A<0 || _m1B<0 || _m2A<0 || _m2B<0) return false;
  if (_e1A<0 || _e1B<0 || _e2A<0 || _e2B<0) return false;

  _self = this;

  // 주기/Δt 설정
  if (period_ms == 0) period_ms = 1;         // 최소 1ms
  _period_ticks = pdMS_TO_TICKS(period_ms);
  _dt_s = period_ms / 1000.0f;

  // 엔코더 인터럽트
  attachInterrupt(digitalPinToInterrupt(_e1A), _enc1ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(_e2A), _enc2ISR, RISING);

  // PID 태스크 생성
  xTaskCreatePinnedToCore(_taskEntry, "KaeriPID", 4096, this, 4, &_task, APP_CPU_NUM);

  return true;
}

void KaeriMotorDriver::PIDset(float kp, float ki, float kd){
  _kp=kp; _ki=ki; _kd=kd;
}

void KaeriMotorDriver::MotorSetSpeed(int m1_rpm, int m2_rpm){
  _m1Target = m1_rpm;
  _m2Target = m2_rpm;
}

// --- 내부 PID 스텝 (주기마다 호출) ---
void KaeriMotorDriver::_pidStep(){
  // 1) 엔코더 스냅샷
  noInterrupts();
  long m1snap = _m1pos_i;
  long m2snap = _m2pos_i;
  interrupts();

  // 2) 위치 증분 → 속도 → RPM
  long m1posT = m1snap - _m1posPrev; _m1posPrev = m1snap;
  long m2posT = m2snap - _m2posPrev; _m2posPrev = m2snap;

  const float dt = _dt_s;
  float m1vel = (float)m1posT / dt;
  float m2vel = (float)m2posT / dt;

  float m1rpm = (m1vel / TICKS_PER_REV) * 60.0f;
  float m2rpm = (m2vel / TICKS_PER_REV) * 60.0f;

  // 3) 간단 LPF
  _m1rpmf = RPM_LPF_A*_m1rpmf + RPM_LPF_B*m1rpm + RPM_LPF_B*_m1rpmprev;
  _m2rpmf = RPM_LPF_A*_m2rpmf + RPM_LPF_B*m2rpm + RPM_LPF_B*_m2rpmprev;
  _m1rpmprev = m1rpm; _m2rpmprev = m2rpm;

  // 4) PID
  float e1 = (float)_m1Target - _m1rpmf;
  float e2 = (float)_m2Target - _m2rpmf;

  _m1int += e1 * dt;
  _m2int += e2 * dt;

  float d1 = (e1 - _m1lerr) / dt;
  float d2 = (e2 - _m2lerr) / dt;

  float u1 = _kp*e1 + _ki*_m1int + _kd*d1;
  float u2 = _kp*e2 + _ki*_m2int + _kd*d2;

  _m1lerr = e1;
  _m2lerr = e2;

  // 5) 포화 & 출력 (analogWrite 사용)
  u1 = constrain(u1, -255.0f, 255.0f);
  u2 = constrain(u2, -255.0f, 255.0f);

  int d1u = (int)fabsf(u1);
  if (u1 <= 0) { analogWrite(_m1A, d1u); analogWrite(_m1B, 0); }
  else         { analogWrite(_m1A, 0);   analogWrite(_m1B, d1u); }

  int d2u = (int)fabsf(u2);
  if (u2 <= 0) { analogWrite(_m2A, d2u); analogWrite(_m2B, 0); }
  else         { analogWrite(_m2A, 0);   analogWrite(_m2B, d2u); }
}
