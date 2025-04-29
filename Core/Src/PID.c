#include "PID.h"
#include "Core.h"

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->previous_error = 0;
    pid->integral = 0;
}

float PID_Compute(PID_t *pid, float setpoint, float measured, float dt) {
    float error = setpoint - measured;
    
    // Anti-windup: Limit integral accumulation
    pid->integral += error * dt;
    if (pid->integral > PID_MAX_INTEGRAL) pid->integral = PID_MAX_INTEGRAL;
    else if (pid->integral < -PID_MAX_INTEGRAL) pid->integral = -PID_MAX_INTEGRAL;
    
    // Calculate derivative with noise filtering
    float derivative = dt > 0.0f ? (error - pid->previous_error) / dt : 0.0f;
    pid->previous_error = error;
    
    // Use PID_E_FACTOR from header
    return PID_E_FACTOR * (
        (pid->Kp * error) + 
        (pid->Ki * pid->integral) + 
        (pid->Kd * derivative)
    );
}

