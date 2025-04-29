#ifndef __PID_H
#define __PID_H

// Define PID constants
#define PID_MAX_INTEGRAL 1000.0f
#define PID_E_FACTOR 0.001f      // Move E_Factor here for global access

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float previous_error;
    float integral;
} PID_t;

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd);
float PID_Compute(PID_t *pid, float setpoint, float measured, float dt);

#endif /* __PID_H */