/**
 * 1-DoF Copter control
 * 
 * Header file for helper.cpp
 * 
 * Date: July 25, 2021
 * Authors: zychosen, nitishbhat09, ShreyasRkk
 * License: 0BSD
 */

#ifndef PID__H
#define PID__H

class PID {

private:
        float *input;
        float *setpoint;

        float Kp;
        float Ki;
        float Kd;

        float kp_d, ki_d, kd_d;

        float min;
        float max;
        float maxInt;
        float minInt;
        float beta;
        float pOnEKp;
        float pOnPVKp;
        bool pON_PV;
        bool pON_E;
        

        float T;

        float integral;
        float derivative;
        float previousReading;
        float previousError;
        unsigned long lastTime;

        float *output;
public:
    PID(float *, float *, float *, float, float, float, int);
    void compute();
    void SetGains(float , float , float, float);
    void limitOutput(float , float);
    float GetKp();
    float GetKi();
    float GetKd();
    float GetBeta();
};
#endif