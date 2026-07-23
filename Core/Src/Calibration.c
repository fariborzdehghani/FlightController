#include "Calibration.h"
#include "MPU6050.h"
#include "Logger.h"
#include "Tools.h"
#include "Core.h"
#include <math.h>

// Filter yaw measurement (wrap-aware low-pass, efficient - no trig)
double FilterYaw(double yaw_meas_deg)
{
    static double prev = 0.0;
    static int inited = 0;
    const double alpha = 0.8; // smoothing: closer to 1 -> slower, smoother

    if (!inited) {
        prev = yaw_meas_deg;
        inited = 1;
        return prev;
    }

    // shortest signed difference from prev to meas in degrees
    double delta = fmod((yaw_meas_deg - prev + 540.0), 360.0) - 180.0;

    // update using low-pass on the difference (avoids trig functions)
    prev += (1.0 - alpha) * delta;

    // normalize to [0,360)
    if (prev < 0.0)
        prev += 360.0;
    else if (prev >= 360.0)
        prev = fmod(prev, 360.0);

    return prev;
}

// void Calibrate_MPU6050(void) {
//     LogInformation(1001, "Starting MPU6050 calibration...");
    
//     // Collect multiple samples for averaging
//     const int numSamples = 200;
//     float roll_sum = 0, pitch_sum = 0;
    
//     for(int i = 0; i < numSamples; i++) {
//         MPU6050_ReadAll(&MPU6050);
//         roll_sum += MPU6050.KalmanAngleY;
//         pitch_sum += MPU6050.KalmanAngleX;
//         HAL_Delay(10);
//     }
    
//     // Calculate average offsets
//     // MPU6050_Roll_Offset = roll_sum / numSamples;
//     // MPU6050_Pitch_Offset = pitch_sum / numSamples;
    
//     LogInformation(1001, "MPU6050 calibration completed");
// }
