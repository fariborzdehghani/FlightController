#ifndef __CORE_H
#define __CORE_H

#include "main.h"
#include "PID.h"

#ifdef __cplusplus
extern "C" {
#endif

extern SPI_HandleTypeDef hspi2;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

#define PACKET_LENGTH      64
#define LED_SHORT_DELAY    200
#define LED_LONG_DELAY     1000
#define LED_READY_BLINKS   5
#define LED_CONFIG_BLINKS  2
#define LED_ARMED_BLINKS   3
#define LED_FLIGHT_BLINKS  4
#define COMMAND_FAILSAFE_TIMEOUT_MS 1000u

/* Set to 0 (or override with -DHEARTBEAT_FAILSAFE_ENABLED=0) to disable
 * automatic disarming when heartbeat/control packets stop arriving. */
#ifndef HEARTBEAT_FAILSAFE_ENABLED
#define HEARTBEAT_FAILSAFE_ENABLED 1u
#endif

#if (HEARTBEAT_FAILSAFE_ENABLED != 0u) && \
    (HEARTBEAT_FAILSAFE_ENABLED != 1u)
#error "HEARTBEAT_FAILSAFE_ENABLED must be 0 or 1"
#endif

#define PACKET_TYPE_CONFIGURATION 1u
#define PACKET_TYPE_CONTROL       2u
#define PACKET_TYPE_HEARTBEAT     3u
#define CONTROL_COMMAND_DISARM    0u
#define CONTROL_COMMAND_ARM       1u
#define CONTROL_COMMAND_TAKEOFF   2u
#define TAKEOFF_ALTITUDE_DEFAULT_CM 50.0
#define TAKEOFF_ALTITUDE_MIN_CM     20.0
#define TAKEOFF_ALTITUDE_MAX_CM     300.0
#define TAKEOFF_MIN_CLIMB_CM        5.0
#define TAKEOFF_THROTTLE_RAMP_DEFAULT_PER_SECOND 10.0
#define TAKEOFF_THROTTLE_RAMP_MIN_PER_SECOND      1.0
#define TAKEOFF_THROTTLE_RAMP_MAX_PER_SECOND     30.0
#define TAKEOFF_MAX_TILT_DEG                      15.0

// Safety constants
#define MOTOR_ABSOLUTE_MIN_THROTTLE 0.0f
#define MOTOR_ABSOLUTE_MAX_THROTTLE 100.0f

// State Manager
typedef struct {
    double roll_deg;           // Current roll angle in degrees
    double pitch_deg;          // Current pitch angle in degrees
    double yaw_deg;            // Current yaw angle in degrees
    double roll_rad;           // Current roll angle in radians
    double pitch_rad;          // Current pitch angle in radians
    double yaw_rad;            // Current yaw angle in radians
    double Gz;                 // Gyroscope Z-axis from MPU6050
    double Vz;                 // Estimated world Z-axis velocity in m/s (positive upward)
    uint8_t Vz_valid;          // 1 when the vertical-velocity estimate is usable
    double fly_altitude;       // Current altitude read from barometer
    double takeoff_altitude;   // Current altitude read from distance sensor
    uint8_t takeoff_altitude_valid; // 1 when the distance-sensor altitude is usable
} FlightData_t;

typedef enum {
    DRONE_STATE_INIT,
    DRONE_STATE_CONFIGURED,
    DRONE_STATE_ARMED,
    DRONE_STATE_TAKEOFF,
    DRONE_STATE_FLYING,
    DRONE_STATE_FAULT
} DroneState_t;

typedef struct {
    DroneState_t state;
    uint32_t lastLedUpdate;
    uint8_t ledBlinkCounter;
    uint8_t ledIsOn;
} DroneStateManager_t;

// Add motor speed structure definition
typedef struct {
    double front_left;   // Motor 1
    double front_right;  // Motor 2
    double back_right;   // Motor 3
    double back_left;    // Motor 4
} MotorSpeeds_t;

// Configuration structures
typedef struct {
    double Kp;
    double Ki;
    double Kd;
    int16_t target; // Changed from double to int16_t for consistency
} PIDConfig_t;

typedef struct {
    double minThrottle;
    double maxThrottle;
    double maxAngle;
    double armThrottle;
    double hoverThrottle;
    double takeoffThrottleRampPerSecond;
    double takeoffAltitudeCm;
    double pidMaxIPart;
    double pidMaxOutput;    // Add this new field
    PIDConfig_t pitch;
    PIDConfig_t roll;
    PIDConfig_t yaw;
    PIDConfig_t Gz;       // PID config for yaw control based on gyroscope Z-axis
    PIDConfig_t altitude;   // Altitude PID gains
} DroneConfig_t;

void Core_init(void);
void Core_loop(void);
void HandlePackage(const uint8_t *data, uint8_t length);
void apply_pid_config(void);
void UpdateLEDState(void);

extern DroneConfig_t Config;
extern DroneStateManager_t droneState;
extern MotorSpeeds_t Motors_Speed;
extern PID_t pid_roll;  // PID controller for roll axis
extern PID_t pid_pitch; // PID controller for pitch axis
extern PID_t pid_Gz;    // PID controller for Gz (angular velocity) axis
extern PID_t pid_altitude; // PID controller for SRF05 altitude
extern FlightData_t flightData;

#ifdef __cplusplus
}
#endif

#endif /* __CORE_H */
