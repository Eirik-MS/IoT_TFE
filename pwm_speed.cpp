// Measure frequency and duty cycle on any digital pin using pulseIn.

#include <Arduino.h>
#include "pwm_speed.h"

volatile unsigned long lastRise = 0;
volatile unsigned long lastPeriod = 0;
volatile unsigned long lastHigh = 0;
volatile bool          havePeriod = false;
volatile uint8_t       lastState;

WaveStats measureWave(uint8_t pin, unsigned long timeout_us = 500000UL) {
  // sync to a clean edge to reduce partial-cycle reads
  pulseIn(pin, HIGH, timeout_us); 
  unsigned long tHigh = pulseIn(pin, HIGH, timeout_us);
  unsigned long tLow  = pulseIn(pin, LOW,  timeout_us);

  WaveStats r{};
  if (tHigh == 0 || tLow == 0) { 
    r.valid = false;
    return r; 
    }

  unsigned long period = tHigh + tLow; // microseconds
  r.frequency_hz = 1e6f / (float)period;
  r.duty_percent = 100.0f * (float)tHigh / (float)period;
  r.valid = true;
  return r;
};

void onEdge() {
  unsigned long now = micros();
  uint8_t s = digitalRead(SIGNAL_PIN_PWM);

  if (s == HIGH) {
    // Rising edge
    if (lastRise != 0) {
      lastPeriod = now - lastRise;  
      havePeriod = true;
    }
    lastRise = now;
  } else {
    // Falling edge
    if (lastRise != 0) {
      lastHigh = now - lastRise;    
    }
  }
  lastState = s;
}

/*
void setup_pwmITR(){
  pinMode(SIGNAL_PIN_PWM, INPUT);
  attachInterrupt(digitalPinToInterrupt(SIGNAL_PIN_PWM), onEdge, CHANGE);
}*/

WaveStats readWave() {
  WaveStats r{};
  noInterrupts();
  unsigned long p = lastPeriod;
  unsigned long h = lastHigh;
  bool ok = havePeriod && p > 0 && h <= p;
  interrupts();

  if (!ok) { r.valid = false; return r; }
  r.frequency_hz = 1e6f / (float)p;
  r.duty_percent = 100.0f * (float)h / (float)p;
  r.valid = true;
  return r;
}

Motion waveToMotion(const WaveStats& w, float speedScale = 0.5f, float accelMaxG  = 4.0f)
{
  Motion m{};
  if (!w.valid) {
    m.speed = 0;
    m.accs = 0;
    return m;
  }

  // Clamp duty to sane bounds
  float duty = w.duty_percent;
  if (duty < 0.0f) duty = 0.0f;
  if (duty > 100.0f) duty = 100.0f;

  m.speed      = w.frequency_hz * speedScale;
  m.accs = (duty / 100.0f) * accelMaxG * 9.80665f;   // standard gravity
  return m;
}
