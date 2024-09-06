#include "PID.h"

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->previous_error = 0;
    pid->integral = 0;
}

float PID_Compute(PID_t *pid, float setpoint, float measured, float dt) {
    float E_Factor = 0.001;
    float error = setpoint - measured;
    pid->integral += error * dt;
    float derivative = (error - pid->previous_error) / dt;
    pid->previous_error = error;
    return E_Factor * ((pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * derivative));
}

