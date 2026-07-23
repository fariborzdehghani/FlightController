#include "PID.h"
#include "Core.h"
#include <math.h>

void PID_Init(PID_t *pid, double Kp, double Ki, double Kd) {
    if (pid == NULL) {
        return;
    }
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->previous_error = 0.0f;
    pid->previous_error_valid = 0U;
    pid->integral = 0.0f;
    pid->max_i_term = Config.pidMaxIPart;        // Use max from config
    pid->min_i_term = -Config.pidMaxIPart;       // Use negative max from config
    pid->max_output = Config.pidMaxOutput;        // Use max output from config
    pid->min_output = -Config.pidMaxOutput;       // Use negative max output from config
}

double PID_Calculate(PID_t *pid, double error, double dt) {
    if (pid == NULL || !isfinite(error) || !isfinite(dt) || dt <= 0.0) {
        return 0.0;
    }

    // Proportional term
    pid->p_term = pid->Kp * error;
    
    // Avoid a derivative kick on the first update after a reset.
    if (pid->previous_error_valid) {
        const double derivative = (error - pid->previous_error) / dt;
        pid->d_term = pid->Kd * derivative;
    } else {
        pid->d_term = 0.0;
        pid->previous_error_valid = 1U;
    }
    
    // Integral term with anti-windup
    if (pid->Ki == 0.0) {
        pid->integral = 0.0;
        pid->i_term = 0.0;
    } else {
        pid->integral += error * dt;
        pid->i_term = pid->Ki * pid->integral;
    }
    
    // Clamp integral term
    if (pid->i_term > pid->max_i_term) {
        pid->i_term = pid->max_i_term;
        pid->integral = (pid->Ki != 0.0) ? pid->max_i_term / pid->Ki : 0.0;
    } 
    else if (pid->i_term < pid->min_i_term) {
        pid->i_term = pid->min_i_term;
        pid->integral = (pid->Ki != 0.0) ? pid->min_i_term / pid->Ki : 0.0;
    }
    
    // Calculate total output
    double output = pid->p_term + pid->i_term + pid->d_term;
    
    // Clamp total output
    if (output > pid->max_output) {
        output = pid->max_output;
    }
    else if (output < pid->min_output) {
        output = pid->min_output;
    }
       
    // Save error for next iteration
    pid->previous_error = error;

    return output;
}

void PID_Reset(PID_t *pid) {
    if (pid == NULL) {
        return;
    }
    pid->previous_error = 0.0f;
    pid->previous_error_valid = 0U;
    pid->integral = 0.0f;
    pid->p_term = 0.0f;
    pid->i_term = 0.0f;
    pid->d_term = 0.0f;
}

void PID_ClearTerms(PID_t *pid) {
    if (pid == NULL) {
        return;
    }
    pid->p_term = 0.0f;
    pid->i_term = 0.0f;
    pid->d_term = 0.0f;
}

