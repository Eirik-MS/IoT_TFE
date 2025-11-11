#pragma once

#include <RIC3D.h>

#define SIGNAL_PIN_PWM DI0

struct WaveStats {
  float frequency_hz;   // 0 if no signal within timeout
  float duty_percent;   // 0..100
  bool  valid;
};

struct Motion {
  float speed;
  float accs;
};

//****************************************************//
// Meassure wave uses arduinos own pulseIN library, 
// its fully blocking and does not work on very high frequensies 
// 
// measure Wave needs PIN to be set as input before use
WaveStats measureWave(uint8_t pin, unsigned long timeout_us = 500000UL);



//*****************************************************//
// setup_pwmITR and read wave uses intterup based code to meassure the frequency of the signal and need to be used together.
// use the setup function in the arduino setup part and readWave in the loop part
// the IQR uses SIGNAL_PIN_PWM

//void setup_pwmITR();

WaveStats readWave();


//**************************************************//
Motion waveToMotion(const WaveStats& w, float speedScale = 0.5f, float accelMaxG  = 4.0f);



