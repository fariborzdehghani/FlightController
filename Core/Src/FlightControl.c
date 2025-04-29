#include "FlightControl.h"
#include "MPU6050.h"
#include "BMPXX80.h"
#include "MotorControl.h"
#include "Tools.h"
#include "Core.h"

void FlightControl_Init(void) {
    // Reset PID states without changing gains
    pid_roll.previous_error = 0;
    pid_roll.integral = 0;
    pid_pitch.previous_error = 0;
    pid_pitch.integral = 0;
    pid_yaw.previous_error = 0;
    pid_yaw.integral = 0;
    pid_Vz.previous_error = 0;
    pid_Vz.integral = 0;
}

void FlightControl_Update(double dt) {
    // Read sensors
    MPU6050_ReadAll(&MPU6050);
    float current_altitude = BMP280_ReadAltitude(101325);

    // Update global state
    angle_x = MPU6050.KalmanAngleX;
    angle_y = MPU6050.KalmanAngleY;
    angle_z = MPU6050.Gz;
    altitude = current_altitude;
    vertical_velocity = MPU6050.Vz;

    // Compute PID values with bounds checking
    double pid_roll_value = PID_Compute(&pid_roll, Config.roll.target, angle_x, dt);
    double pid_pitch_value = PID_Compute(&pid_pitch, Config.pitch.target, angle_y, dt);
    double pid_yaw_value = PID_Compute(&pid_yaw, Config.yaw.target, angle_z, dt);
    double pid_vz_value = PID_Compute(&pid_Vz, Config.vz.target, vertical_velocity, dt);

    // Update throttle with bounds checking
    Config.throttle = Config.throttle + pid_vz_value;
    if (Config.throttle < Config.minSpeed) Config.throttle = Config.minSpeed;
    if (Config.throttle > Config.maxSpeed) Config.throttle = Config.maxSpeed;

    // Calculate motor speeds using direct struct access
    Motors_Speed.front_left = Config.throttle - pid_roll_value - pid_pitch_value - pid_yaw_value;
    Motors_Speed.front_right = Config.throttle + pid_roll_value - pid_pitch_value + pid_yaw_value;
    Motors_Speed.back_left = Config.throttle - pid_roll_value + pid_pitch_value + pid_yaw_value;
    Motors_Speed.back_right = Config.throttle + pid_roll_value + pid_pitch_value - pid_yaw_value;

    // Apply motor constraints using helper function
    Motors_ClampSpeeds(&Motors_Speed, Config.minSpeed, Config.maxSpeed);

    // Update motors (pass struct directly)
    Motors_SetSpeed(Motors_Speed);
}

void FlightControl_Stop(void) {
    Config.throttle = 0;
    Motors_Reset(&Motors_Speed);
    Motors_SetSpeed(Motors_Speed);
}

void FlightControl_Start(void) {
    Config.throttle = Config.minSpeed;
    FlightControl_Init();    // Only reset PID states
}
