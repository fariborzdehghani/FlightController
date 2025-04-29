#ifndef __CORE_H
#define __CORE_H

#include "main.h"
#include "MPU6050.h"

#ifdef __cplusplus
extern "C" {
#endif

enum ResponseType
{
    ConfigurationSet = 1,
    DroneStarted = 2,
    DroneStopped = 3
};

// LED Constants moved from Core.c
#define LED_SHORT_DELAY    200
#define LED_LONG_DELAY     1000
#define LED_STARTED_BLINKS 3
#define LED_CONFIG_BLINKS  2

// Safety constants
#define MOTOR_ABSOLUTE_MIN_SPEED 0.0f
#define MOTOR_ABSOLUTE_MAX_SPEED 100.0f

// External handle declarations
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart1;

// External MPU6050 variables
extern uint8_t MPU6050_readystatus;
extern MPU6050_t MPU6050;

// Variables
extern float altitude;
extern double angle_x;
extern double angle_y;
extern double angle_z;
extern double vertical_velocity;
extern double LEDTimer;

// Add motor speed structure definition
typedef struct {
    double front_left;   // Motor 1
    double front_right;  // Motor 2
    double back_left;    // Motor 3
    double back_right;   // Motor 4
} MotorSpeeds_t;

// Configuration structures
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float target;
} PIDConfig_t;

typedef struct {
    float minSpeed;
    float maxSpeed;
    float maxAngle;
    float throttle;    // Add throttle to config
    PIDConfig_t pitch;
    PIDConfig_t roll;
    PIDConfig_t yaw;
    PIDConfig_t vz;
} DroneConfig_t;

// Replace individual state enums with a combined state management
typedef enum {
    DRONE_STATE_INIT,
    DRONE_STATE_CONFIGURED,
    DRONE_STATE_FLYING
} DroneState_t;

typedef struct {
    DroneState_t state;
    uint32_t lastLedUpdate;
    uint8_t ledBlinkCounter;
    uint8_t ledIsOn;
} DroneStateManager_t;

// External configuration
extern DroneConfig_t Config;

// External state manager
extern DroneStateManager_t droneState;

// PID Variables
extern PID_t pid_roll;
extern PID_t pid_pitch;
extern PID_t pid_yaw;
extern PID_t pid_Vz;

// Update motor variables declaration from array to struct
extern MotorSpeeds_t Motors_Speed;

// Functions
void HandlePackage(uint8_t *data);
void Core_init(void);
void Core_loop(void);
void apply_pid_config(void); // Add this declaration
void UpdateLEDState(void); // Update function declaration

// Add these helper function declarations
void Motors_ClampSpeeds(MotorSpeeds_t* speeds, float minSpeed, float maxSpeed);
void Motors_Reset(MotorSpeeds_t* speeds);

#ifdef __cplusplus
}
#endif

#endif /* __CORE_H */
