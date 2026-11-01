#include "Logger.h"
#include "Tools.h"
#include "Core.h"
#include "FlightRecorder.h"
#include "FlightControl.h"
#include "SensorConfig.h"
#include "AppError.h"
#include <string.h>
#include <math.h>

#define LOGGER_INTERVAL_MS FLIGHT_RECORDER_LIVE_INTERVAL_MS

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
    /* Commands retain the legacy slot; recorded configuration identifies auto.
     * Byte 44 is the estimate in half-percent units, updated during flight. */
    accepted_configuration[43] = 0u;
    accepted_configuration[44] = (uint8_t)lround(Config.armThrottle * 2.0);
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
                 (SENSOR_MPU6050_ENABLED ? 16u : 0u) |
                 (d->landing_active ? 32u : 0u),
        .values = {d->roll, d->pitch, d->yaw, d->Gz, d->Vz, d->altitude,
            d->base_throttle, d->m1_speed, d->m2_speed, d->m3_speed, d->m4_speed,
            d->roll_p, d->roll_i, d->roll_d, d->roll_total,
            d->pitch_p, d->pitch_i, d->pitch_d, d->pitch_total,
            d->Gz_p, d->Gz_i, d->Gz_d, d->altitude_p, d->altitude_i, d->altitude_d,
            d->loop_dt_ms, d->altitude_target, d->roll_rate_dps, d->pitch_rate_dps},
        .attitude_trace = d->attitude_trace,
        .srf05_trace = d->srf05_trace
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
    latest_data.attitude_trace = flightData.attitude_trace;
    latest_data.srf05_trace = flightData.srf05_trace;
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
    record.error = (uint16_t)AppError_GetCode();
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
    record.flags &= (uint8_t)~32u; /* Final event is stopped, not actively landing. */
    /* Final event represents the stopped outputs. Keep the preceding PID terms
     * for diagnosis; no controller calculation is performed by the logger. */
    for (unsigned i = 6; i <= 10; i++) record.values[i] = 0.0f;
    FlightRecorder_Record(&record, true);
    FlightRecorder_End(reason, error, hal_status, record.timestamp_ms);
}

static void log_full_serial_sample(const FlightRecorderRecord_t *record)
{
    /* 336-byte v7 snapshot + CRC16 -> 452 Base64 characters. Including
     * LiveLog= and newline: 461 bytes, at most 4610 bytes/s at 115200 8N1.
     * The existing asynchronous UART queue keeps transmission nonblocking. */
    uint8_t packet[FLIGHT_RECORDER_LIVE_BLOB_SIZE + 2u];
    uint8_t config[64];
    memcpy(config, accepted_configuration, sizeof(config));
    const uint16_t yaw = (uint16_t)Config.yaw.target;
    config[9] = (uint8_t)yaw;
    config[10] = (uint8_t)(yaw >> 8);
    config[44] = (uint8_t)lround(FlightControl_GetHoverThrottleEstimate() * 2.0);
    FlightRecorder_EncodeLiveSample(config, record, packet);
    const uint16_t crc = FlightLog_Crc16(packet, FLIGHT_RECORDER_LIVE_BLOB_SIZE);
    packet[FLIGHT_RECORDER_LIVE_BLOB_SIZE] = (uint8_t)crc;
    packet[FLIGHT_RECORDER_LIVE_BLOB_SIZE + 1u] = (uint8_t)(crc >> 8);

    static const char alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    char line[8u + 4u * ((sizeof(packet) + 2u) / 3u) + 1u];
    memcpy(line, "LiveLog=", 8u);
    unsigned out = 8u;
    for (unsigned i = 0u; i < sizeof(packet); i += 3u)
    {
        const uint32_t value = ((uint32_t)packet[i] << 16) |
            (i + 1u < sizeof(packet) ? (uint32_t)packet[i + 1u] << 8 : 0u) |
            (i + 2u < sizeof(packet) ? packet[i + 2u] : 0u);
        line[out++] = alphabet[(value >> 18) & 63u];
        line[out++] = alphabet[(value >> 12) & 63u];
        line[out++] = i + 1u < sizeof(packet) ? alphabet[(value >> 6) & 63u] : '=';
        line[out++] = i + 2u < sizeof(packet) ? alphabet[value & 63u] : '=';
    }
    line[out] = '\0';
    (void)LogStringToPC(line);
}

void Logger_LogFlightData(const FlightLogData_t *log_data) {
    static uint32_t last_log_ms;

    if (log_data == NULL) {
        return;
    }

    latest_data = *log_data;
    FlightRecorder_SetHoverThrottle((float)FlightControl_GetHoverThrottleEstimate());
    FlightRecorderRecord_t record = make_record(log_data, (uint8_t)droneState.state);
    record.error = (uint16_t)AppError_GetCode();
    FlightRecorder_Record(&record, false);
    if (!live_serial) return;

    const uint32_t now = HAL_GetTick();
    if ((now - last_log_ms) < LOGGER_INTERVAL_MS) {
        return;
    }
    last_log_ms = now;

    log_full_serial_sample(&record);
}
