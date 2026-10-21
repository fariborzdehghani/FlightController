#include "Logger.h"
#include "Tools.h"
#include "Core.h"
#include "FlightRecorder.h"
#include "SensorConfig.h"
#include <stdio.h>
#include <string.h>

#define LOGGER_INTERVAL_MS 100u

static uint8_t live_serial;
static uint8_t accepted_configuration[64];
static FlightLogData_t latest_data;

void Logger_SetLiveSerial(uint8_t enabled) { live_serial = enabled; }
void Logger_SetLoopTiming(float dt_ms)
{
    latest_data.loop_dt_ms = dt_ms;
    FlightRecorder_SetLoopTiming(dt_ms);
}

void Logger_SetConfiguration(const uint8_t *data, uint8_t length)
{
    memset(accepted_configuration, 0, sizeof(accepted_configuration));
    memcpy(accepted_configuration, data, length < 64u ? length : 64u);
    /* Store effective defaults as well as the quantized gains actually applied. */
    const uint16_t altitude = (uint16_t)Config.takeoffAltitudeCm;
    accepted_configuration[13] = (uint8_t)altitude;
    accepted_configuration[14] = (uint8_t)(altitude >> 8);
    const uint16_t ramp = (uint16_t)(Config.takeoffThrottleRampPerSecond * 10.0);
    accepted_configuration[41] = (uint8_t)ramp;
    accepted_configuration[42] = (uint8_t)(ramp >> 8);
    /* The automatic hover default can include a half percent. Byte 44 stores
     * hover throttle in half-percent units in the log snapshot only. */
    accepted_configuration[44] = (uint8_t)(Config.hoverThrottle * 2.0);
    accepted_configuration[45] =
        (SENSOR_MPU6050_ENABLED ? 1u : 0u) | (SENSOR_QMC5883_ENABLED ? 2u : 0u) |
        (SENSOR_SRF05_ENABLED ? 4u : 0u) | (SENSOR_BMP280_ENABLED ? 8u : 0u);
}

static FlightRecorderRecord_t make_record(const FlightLogData_t *d, uint8_t state)
{
    FlightRecorderRecord_t record = {
        .timestamp_ms = HAL_GetTick(), .state = state,
        .flags = (d->Vz_valid ? 1u : 0u) |
                 (flightData.takeoff_altitude_valid ? 2u : 0u) |
                 (d->altitude_control_active ? 4u : 0u) |
                 (d->mixer_limited ? 8u : 0u) |
                 (SENSOR_MPU6050_ENABLED ? 16u : 0u),
        .values = {d->roll, d->pitch, d->yaw, d->Gz, d->Vz, d->altitude,
            d->base_throttle, d->m1_speed, d->m2_speed, d->m3_speed, d->m4_speed,
            d->roll_p, d->roll_i, d->roll_d, d->roll_total,
            d->pitch_p, d->pitch_i, d->pitch_d, d->pitch_total,
            d->Gz_p, d->Gz_i, d->Gz_d, d->altitude_p, d->altitude_i, d->altitude_d,
            d->loop_dt_ms, d->altitude_target, d->roll_rate_dps, d->pitch_rate_dps}
    };
    return record;
}

static void update_sensor_snapshot(void)
{
    latest_data.roll = flightData.roll_deg;
    latest_data.pitch = flightData.pitch_deg;
    latest_data.yaw = flightData.yaw_deg;
    latest_data.Gz = flightData.Gz;
    latest_data.roll_rate_dps = flightData.roll_rate_dps;
    latest_data.pitch_rate_dps = flightData.pitch_rate_dps;
    latest_data.Vz = flightData.Vz;
    latest_data.Vz_valid = flightData.Vz_valid;
    latest_data.altitude = flightData.takeoff_altitude;
}

void Logger_PrepareDownload(void)
{
    if (FlightRecorder_HasFlight()) return;
    const float dt_ms = latest_data.loop_dt_ms;
    memset(&latest_data, 0, sizeof(latest_data));
    latest_data.loop_dt_ms = dt_ms;
    update_sensor_snapshot();
    accepted_configuration[45] =
        (SENSOR_MPU6050_ENABLED ? 1u : 0u) | (SENSOR_QMC5883_ENABLED ? 2u : 0u) |
        (SENSOR_SRF05_ENABLED ? 4u : 0u) | (SENSOR_BMP280_ENABLED ? 8u : 0u);
    FlightRecorderRecord_t record = make_record(&latest_data, (uint8_t)droneState.state);
    FlightRecorder_Snapshot(accepted_configuration, &record);
}

void Logger_BeginFlight(void)
{
    const uint16_t yaw_target = (uint16_t)Config.yaw.target;
    accepted_configuration[9] = (uint8_t)yaw_target;
    accepted_configuration[10] = (uint8_t)(yaw_target >> 8);
    FlightRecorder_Begin(accepted_configuration, HAL_GetTick());
    memset(&latest_data, 0, sizeof(latest_data));
    update_sensor_snapshot();
    latest_data.base_throttle = Config.armThrottle;
    latest_data.m1_speed = Motors_Speed.front_left;
    latest_data.m2_speed = Motors_Speed.front_right;
    latest_data.m3_speed = Motors_Speed.back_right;
    latest_data.m4_speed = Motors_Speed.back_left;
    FlightRecorderRecord_t record = make_record(&latest_data, DRONE_STATE_TAKEOFF);
    FlightRecorder_Record(&record, true);
}

void Logger_EndFlight(uint8_t reason, uint16_t error, uint8_t hal_status, uint8_t state)
{
    if (!FlightRecorder_IsRecording()) return;
    update_sensor_snapshot();
    FlightRecorderRecord_t record = make_record(&latest_data, state);
    record.error = error;
    /* Final event represents the stopped outputs. Keep the preceding PID terms
     * for diagnosis; no controller calculation is performed by the logger. */
    for (unsigned i = 6; i <= 10; i++) record.values[i] = 0.0f;
    FlightRecorder_Record(&record, true);
    FlightRecorder_End(reason, error, hal_status, record.timestamp_ms);
}

void Logger_LogFlightData(const FlightLogData_t *log_data) {
    static uint32_t last_log_ms;

    if (log_data == NULL) {
        return;
    }

    latest_data = *log_data;
    FlightRecorderRecord_t record = make_record(log_data, (uint8_t)droneState.state);
    FlightRecorder_Record(&record, false);
    if (!live_serial) return;

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
