#include "Kaeri_MotorDriver.h"

// ===== 사용자 환경에 맞춰 조정 =====
#define TICKS_PER_REV 600.0f       // 엔코더 한바퀴 카운트 (필요시 수정)
#define RPM_LPF_A     0.854f       // 간단 2차 IIR (양쪽 B 동일)
#define RPM_LPF_B     0.0728f
#define DT_FIXED_S    0.005f       // 5ms 고정 샘플링

KaeriMotorDriver* KaeriMotorDriver::_self = nullptr;
KaeriMotorDriver Driver;

// ---------- 엔코더 ISR ----------
void IRAM_ATTR KaeriMotorDriver::_enc1ISR(){
  // A채널 RISING에서 B 읽어 방향 판별
  int b = digitalRead(Driver._self->_e1B);
  Driver._self->_m1pos_i += (b>0) ? -1 : +1;
}
void IRAM_ATTR KaeriMotorDriver::_enc2ISR(){
  int b = digitalRead(Driver._self->_e2B);
  Driver._self->_m2pos_i += (b>0) ? -1 : +1;
}

// ---------- 타이머 ISR (알림만 전송) ----------
void IRAM_ATTR KaeriMotorDriver::_timerISR_tramp(){
  BaseType_t hpw = pdFALSE;
  if (Driver._self && Driver._self->_task){
    vTaskNotifyGiveFromISR(Driver._self->_task, &hpw);
    if (hpw) portYIELD_FROM_ISR();
  }
}

// ---------- PID 태스크 ----------
void KaeriMotorDriver::_taskEntry(void* arg){
  auto* self = static_cast<KaeriMotorDriver*>(arg);
  for(;;){
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // 5ms마다 깨어남
    self->_pidStep();
  }
}

// ---------- 외부 API ----------
void KaeriMotorDriver::MotorSetup(int m1A, int m1B, int m2A, int m2B){
  _m1A=m1A; _m1B=m1B; _m2A=m2A; _m2B=m2B;
  pinMode(_m1A, OUTPUT);
  pinMode(_m1B, OUTPUT);
  pinMode(_m2A, OUTPUT);
  pinMode(_m2B, OUTPUT);
}

void KaeriMotorDriver::EncoderSetup(int enc1A, int enc1B, int enc2A, int enc2B){
  _e1A=enc1A; _e1B=enc1B; _e2A=enc2A; _e2B=enc2B;
  pinMode(_e1A, INPUT);
  pinMode(_e1B, INPUT);
  pinMode(_e2A, INPUT);
  pinMode(_e2B, INPUT);
}

void KaeriMotorDriver::PIDset(float kp, float ki, float kd){
  _kp = kp; _ki = ki; _kd = kd;
}

void KaeriMotorDriver::MotorSetSpeed(int m1_rpm, int m2_rpm){
  _m1Target = m1_rpm;
  _m2Target = m2_rpm;
}

bool KaeriMotorDriver::Begin(uint32_t period_ms){
  if (_m1A<0 || _m1B<0 || _m2A<0 || _m2B<0) return false;
  if (_e1A<0 || _e1B<0 || _e2A<0 || _e2B<0) return false;

  _self = this;
  _period_us = period_ms * 1000UL;

  _setupLEDC();

  // 엔코더 인터럽트(A채널 RISING)
  attachInterrupt(digitalPinToInterrupt(_e1A), _enc1ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(_e2A), _enc2ISR, RISING);

  // PID 태스크 (높은 우선순위, 코어 고정은 필요시 조정)
  xTaskCreatePinnedToCore(_taskEntry, "KaeriPIDTask", 4096, this, 4, &_task, APP_CPU_NUM);

  // 하드웨어 타이머 (1us tick)
  _timer = timerBegin(0, 80, true);            // APB 80MHz / 80 = 1MHz(=1us)
  timerAttachInterrupt(_timer, &_timerISR_tramp, true);
  timerAlarmWrite(_timer, _period_us, true);   // period_us마다 인터럽트
  timerAlarmEnable(_timer);

  return true;
}

// ---------- 내부 구현 ----------
void KaeriMotorDriver::_setupLEDC(){
  ledcSetup(_ch_m1A, _pwm_freq, _pwm_bits);
  ledcSetup(_ch_m1B, _pwm_freq, _pwm_bits);
  ledcSetup(_ch_m2A, _pwm_freq, _pwm_bits);
  ledcSetup(_ch_m2B, _pwm_freq, _pwm_bits);

  ledcAttachPin(_m1A, _ch_m1A);
  ledcAttachPin(_m1B, _ch_m1B);
  ledcAttachPin(_m2A, _ch_m2A);
  ledcAttachPin(_m2B, _ch_m2B);

  ledcWrite(_ch_m1A, 0); ledcWrite(_ch_m1B, 0);
  ledcWrite(_ch_m2A, 0); ledcWrite(_ch_m2B, 0);
}

void KaeriMotorDriver::_writeMotor(int chA, int chB, float u){
  u = constrain(u, -255.f, 255.f);
  const int duty = (int)fabsf(u);

  if (u <= 0){
    ledcWrite(chA, duty);
    ledcWrite(chB, 0);
  } else {
    ledcWrite(chA, 0);
    ledcWrite(chB, duty);
  }
}

void KaeriMotorDriver::_pidStep(){
  // --- 1) 공유 카운트 스냅샷 ---
  long m1pos, m2pos;
  portENTER_CRITICAL(&_mux);
  m1pos = _m1pos_i;
  m2pos = _m2pos_i;
  portEXIT_CRITICAL(&_mux);

  // --- 2) 속도 추정 ---
  long m1d = m1pos - _m1posPrev;
  long m2d = m2pos - _m2posPrev;
  _m1posPrev = m1pos;
  _m2posPrev = m2pos;

  // counts/sec -> rpm : (Δcount / Δt) / TICKS_PER_REV * 60
  const float dt = DT_FIXED_S; // 0.005 s 고정
  float m1vel = (float)m1d / dt;
  float m2vel = (float)m2d / dt;
  float m1rpm = (m1vel / TICKS_PER_REV) * 60.0f;
  float m2rpm = (m2vel / TICKS_PER_REV) * 60.0f;

  // --- 3) 간단 로우패스 필터(2차 IIR)
  _m1rpmf = RPM_LPF_A*_m1rpmf + RPM_LPF_B*m1rpm + RPM_LPF_B*_m1rpmprev;
  _m2rpmf = RPM_LPF_A*_m2rpmf + RPM_LPF_B*m2rpm + RPM_LPF_B*_m2rpmprev;
  _m1rpmprev = m1rpm;
  _m2rpmprev = m2rpm;

  // --- 4) PID ---
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

  // --- 5) 포화 & 출력 ---
  u1 = constrain(u1, -255.f, 255.f);
  u2 = constrain(u2, -255.f, 255.f);

  _writeMotor(_ch_m1A, _ch_m1B, u1);
  _writeMotor(_ch_m2A, _ch_m2B, u2);
}
