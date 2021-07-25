/**
 * 1-DoF Copter control
 * 
 * Creating helper functions to compute PID algorithm, set and access control parameters and limit PID output
 * 
 * Date: July 25, 2021
 * Authors: zychosen, nitishbhat09, ShreyasRkk
 * License: 0BSD
 */

#include "pid.h"
#include <Arduino.h>

/* PID constructor */
PID::PID(float *Input, float * out, float *sp, float kp, float ki, float kd, int sampleTime) {
    input = Input;
    setpoint = sp;
    beta = 0.55;
    maxInt = 235;
    minInt = 0;
    T = sampleTime;
    PID::SetGains(kp, ki, kd, beta);
    integral = 0.0;

	previousReading = 0.0;
    previousError = 0.0;

	output = out;
    lastTime = millis() - T;
}

/* Modified PID algorithm computation */
void PID::compute() {

    unsigned long now = millis();
    unsigned long timeChange = (now - lastTime);
    if(timeChange >= T)                         // Compute PID only if time T has elapsed
   {
    float out, in = *input, sp = *setpoint;
    float inDiff = in - previousReading;
 
    int16_t error = sp - in;                   // compute error term

    integral += Ki*error;                      // compute integral term

    if (pON_PV) integral -= pOnPVKp*inDiff;    // compute proportional on PV term

    /* limit integral term to prevent integral windup */
    if (integral > maxInt) { 
        integral = maxInt; 
    } else if (integral < minInt) { 
        integral = minInt;
    }

    if (pON_E) out = pOnEKp*error;             // compute proportional on error term
    else out = 0;

    derivative = -Kd*(inDiff);                 // compute derivative term

    out += (int)(integral + derivative);       // compute PID output

    /* Limit output between 0 and 255 */
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

/* Function to set PID gains */
void PID::SetGains(float kp, float ki, float kd, float b) {
    if (Kp<0 || Ki<0|| Kd<0 || beta<0 || beta>1) return;

    beta = b;
    pON_E = beta>0;
    pON_PV = beta<1;

    kp_d = kp; ki_d = ki; kd_d = kd;

    /* rescale the gains to avoid repeated multiplication and division in compute() */
    float s =  (float)T/1000;
    Kp = kp;
    Kd = kd/s;
    Ki = ki*s;

    /* compute proportional on PV and proportional on error terms */
    pOnEKp = beta * kp; 
    pOnPVKp = (1 - beta) * kp;
}

/* Function to limit PID output */
void PID::limitOutput(float Min, float Max) {
    if(Min >= Max) return;
    min = Min;
    max = Max;
}

/* Functions to get current gains */
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
