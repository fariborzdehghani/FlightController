#include "Logger.h"
#include "Tools.h"
#include <stdio.h>
#include <string.h>

#define LOGGER_INTERVAL_MS 100u

void Logger_LogFlightData(const FlightLogData_t *log_data) {
    static uint32_t last_log_ms;

    if (log_data == NULL) {
        return;
    }

    const uint32_t now = HAL_GetTick();
    if ((now - last_log_ms) < LOGGER_INTERVAL_MS) {
        return;
    }
    last_log_ms = now;

    char buffer[512];
    memset(buffer, 0, sizeof(buffer));

    const int written = snprintf(buffer, sizeof(buffer),
             "Data={\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f,\"Gz\":%.2f,\"Vz\":%.2f,\"Vz_valid\":%u,\"altitude\":%.2f,\"base_throttle\":%.2f,"
             "\"m1\":%.2f,\"m2\":%.2f,\"m3\":%.2f,\"m4\":%.2f,"
             "\"roll_p\":%.4f,\"roll_i\":%.4f,\"roll_d\":%.4f,\"roll_total\":%.4f,"
             "\"pitch_p\":%.4f,\"pitch_i\":%.4f,\"pitch_d\":%.4f,\"pitch_total\":%.4f,"
             "\"Gz_p\":%.4f,\"Gz_i\":%.4f,\"Gz_d\":%.4f,"
             "\"altitude_p\":%.4f,\"altitude_i\":%.4f,\"altitude_d\":%.4f}",
             (double)log_data->roll, (double)log_data->pitch, (double)log_data->yaw, (double)log_data->Gz, (double)log_data->Vz, (unsigned int)log_data->Vz_valid, (double)log_data->altitude, (double)log_data->base_throttle,
             (double)log_data->m1_speed, (double)log_data->m2_speed, (double)log_data->m3_speed, (double)log_data->m4_speed,
             (double)log_data->roll_p, (double)log_data->roll_i, (double)log_data->roll_d, (double)log_data->roll_total,
             (double)log_data->pitch_p, (double)log_data->pitch_i, (double)log_data->pitch_d, (double)log_data->pitch_total,
             (double)log_data->Gz_p, (double)log_data->Gz_i, (double)log_data->Gz_d,
             (double)log_data->altitude_p, (double)log_data->altitude_i, (double)log_data->altitude_d);

    if (written > 0 && (size_t)written < sizeof(buffer)) {
        (void)LogStringToPC(buffer);
    }
}
