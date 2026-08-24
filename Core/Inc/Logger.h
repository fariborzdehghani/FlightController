#ifndef __LOGGER_H
#define __LOGGER_H

#include "main.h"

typedef struct {
    float roll;
    float pitch;
    float yaw;
    float Gz;
    float Vz;
    uint8_t Vz_valid;
    float altitude;
    float base_throttle;
    float m1_speed;
    float m2_speed;
    float m3_speed;
    float m4_speed;
    float roll_p;
    float roll_i;
    float roll_d;
    float roll_total;
    float pitch_p;
    float pitch_i;
    float pitch_d;
    float pitch_total;
    float Gz_p;
    float Gz_i;
    float Gz_d;
    float altitude_p;
    float altitude_i;
    float altitude_d;
} FlightLogData_t;

void Logger_LogFlightData(const FlightLogData_t *log_data);

#endif /* __LOGGER_H */
