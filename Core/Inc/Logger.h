#ifndef __LOGGER_H
#define __LOGGER_H

#include "main.h"
#include "AttitudeTrace.h"
#include "Srf05Trace.h"

typedef struct {
    float roll;
    float pitch;
    float yaw;
    float Gz;
    float roll_rate_dps;
    float pitch_rate_dps;
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
    float loop_dt_ms;
    float altitude_target;
    uint8_t altitude_control_active;
    uint8_t mixer_limited;
    uint8_t landing_active;
    AttitudeTrace_t attitude_trace;
    Srf05Trace_t srf05_trace;
} FlightLogData_t;

void Logger_LogFlightData(const FlightLogData_t *log_data);
void Logger_SetConfiguration(const uint8_t *data, uint8_t length);
void Logger_BeginFlight(void);
void Logger_EndFlight(uint8_t reason, uint16_t error, uint8_t hal_status, uint8_t state);
void Logger_SetLiveSerial(uint8_t enabled);
void Logger_SetLoopTiming(float dt_ms);
void Logger_PrepareDownload(void);

#endif /* __LOGGER_H */
