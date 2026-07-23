#ifndef __PID_H
#define __PID_H

#include <stdint.h>

typedef struct {
    double Kp;
    double Ki;
    double Kd;
    double previous_error;
    uint8_t previous_error_valid;
    double integral;
    double max_i_term;  // Maximum integral term
    double min_i_term;  // Minimum integral term
    double max_output;  // Maximum total output
    double min_output;  // Minimum total output
    double p_term;        // Store P term
    double i_term;        // Store I term
    double d_term;        // Store D term
} PID_t;

void PID_Init(PID_t* pid, double kp, double ki, double kd);
double PID_Calculate(PID_t* pid, double error, double dt);
void PID_Reset(PID_t* pid);
void PID_ClearTerms(PID_t* pid);  // Add this line

#endif /* __PID_H */
