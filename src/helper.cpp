#include "pid.h"
#include <Arduino.h>

PID::PID(double *Input, double * out, double *sp, double kp, double ki, double kd, int sampleTime) {
    input = Input;
    setpoint = sp;
    beta = 0.2;
    PID::limitOutput(0,255);
    T = sampleTime;
    PID::SetGains(kp, ki, kd, beta);
    integral = 0.0;

	previousReading = 0.0;
    lastOutput = 0.0;

	output = out;
    lastTime = millis() - T;
}

void PID::compute() {

    unsigned long now = millis();
    unsigned long timeChange = (now - lastTime);
    double proportional;
    if(timeChange >= T)
   {
    double out, in = *input, sp = *setpoint;
    double inDiff = in - previousReading;
    int16_t error = sp - in;

    proportional = Kp*error;
    integral += Ki*error;
    
    if (integral > max) integral = max;
    else if (integral < min) integral = min;

    derivative = -Kd*inDiff;

    out = (int) (proportional + integral + derivative);

    if (out > max) {
        out = max;
    } else if (out < min) {
        out = min;
    }
    
    *output = out;
    lastOutput = out;
    previousReading = in;
    lastTime = now;
  } else return;

}

void PID::SetGains(double kp, double ki, double kd, double b) {
    if (Kp<0 || Ki<0|| Kd<0 || beta<0 || beta>1) return;
    beta = b;

    kp_d = kp; ki_d = ki; kd_d = kd;
    double s =  (double)T/1000;
    Kp = kp;
    Kd = kd/s;
    Ki = ki*s;
}

void PID::limitOutput(double Min, double Max) {
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
