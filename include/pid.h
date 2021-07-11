#ifndef PID__H
#define PID__H

#include <stdint.h>

class PID {

private:
        double *input;
        double *setpoint;

        double Kp;
        double Ki;
        double Kd;

        double kp_d, ki_d, kd_d;

        double min;
        double max;
        double beta;
        

        double T;

        double integral;
        double derivative;
        double previousReading;
        double previousError;
        double lastOutput;
        unsigned long lastTime;

        double *output;
public:
    PID(double *, double *, double *, double, double, double, int);
    void compute();
    void SetGains(double , double , double, double);
    void limitOutput(double , double);
    float GetKp();
    float GetKi();
    float GetKd();
    float GetBeta();
};
#endif