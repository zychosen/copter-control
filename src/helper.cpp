#include "pid.h"
#include <Arduino.h>

PID::PID(float *Input, float * out, float *sp, float kp, float ki, float kd, int sampleTime) {
    input = Input;
    setpoint = sp;
    beta = 0.2;
    T = sampleTime;
    PID::SetGains(kp, ki, kd, beta);
    integral = 0.0;

	previousReading = 0.0;
    previousError = 0.0;

	output = out;
    lastTime = millis() - T;
}

void PID::compute() {

    unsigned long now = millis();
    unsigned long timeChange = (now - lastTime);
    float proportional;
    if(timeChange >= T)
   {
    float out, in = *input, sp = *setpoint;
    float inDiff = in - previousReading;
    int16_t error = sp - in;

    proportional = Kp*error;
    integral += Ki*error;
    if (integral > 245) integral = 245;
    else if (integral < min) integral = min;
    derivative = -Kd*(inDiff);

    out = (int) (proportional + integral + derivative);

    if (out > max) {
        out = max;
    } else if (out < min) {
        out = min;
    }
    
    *output = out;
    previousReading = in;
    previousError = error;
    lastTime = now;
  } else return;

}

void PID::SetGains(float kp, float ki, float kd, float b) {
    if (Kp<0 || Ki<0|| Kd<0 || beta<0 || beta>1) return;
    beta = b;

    kp_d = kp; ki_d = ki; kd_d = kd;
    float s =  (float)T/1000;
    Kp = kp;
    Kd = kd/s;
    Ki = ki*s;
}

void PID::limitOutput(float Min, float Max) {
    if(Min >= Max) return;
    min = Min;
    max = Max;
}

float PID::GetKp() {
    return  kp_d;
}

float PID::GetKi() {
    return ki_d;
}

float PID::GetKd() {
    return  kd_d;
}

float PID::GetBeta() {
    return beta;
}
