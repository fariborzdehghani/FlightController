#include "General.h"
#include "Tools.h"
#include "MotorControl.h"
#include "string.h"
#include "NRF24L01P.h"

void HandlePackage(uint8_t *data)
{
    if (data[0] == 1)
    {
        Config_BaseSpeed = data[1];
        Config_MinSpeed = data[2];
        Config_MaxSpeed = data[3];
        Config_MaxAngle = data[4];
        Config_TargetPitch = data[5];
        Config_TargetRoll = data[6];
        Config_TargetYaw = data[7];
        Config_TargetVz = (float)data[8] / 100.0; // Vz is scaled by 100

        // Pitch PID
        Config_PitchKp = (float)data[9] / 10.0;                         // Kp is scaled by 10
        Config_Pitchki = (float)((data[11] << 8) | data[10]) / 1000.0;  // Ki is scaled by 1000
        Config_PitchKd = (float)((data[13] << 8) | data[12]) / 10000.0; // Kd is scaled by 10000

        // Roll PID
        Config_RollKp = (float)data[15] / 10.0;                        // Kp is scaled by 10
        Config_RollKi = (float)((data[17] << 8) | data[16]) / 1000.0;  // Ki is scaled by 1000
        Config_RollKd = (float)((data[19] << 8) | data[18]) / 10000.0; // Kd is scaled by 10000

        // Yaw PID
        Config_YawKp = (float)data[20] / 10.0;                        // Kp is scaled by 10
        Config_YawKi = (float)((data[22] << 8) | data[21]) / 1000.0;  // Ki is scaled by 1000
        Config_YawKd = (float)((data[24] << 8) | data[23]) / 10000.0; // Kd is scaled by 10000

        // Vertical (Vz) PID
        Config_VzKp = (float)data[25] / 10.0;                        // Kp is scaled by 10
        Config_VzKi = (float)((data[27] << 8) | data[26]) / 1000.0;  // Ki is scaled by 1000
        Config_VzKd = (float)((data[29] << 8) | data[28]) / 10000.0; // Kd is scaled by 10000

        // Init PID
        PID_Init(&pid_pitch, Config_PitchKp, Config_Pitchki, Config_PitchKd);
        PID_Init(&pid_roll, Config_RollKp, Config_RollKi, Config_RollKd);
        PID_Init(&pid_yaw, Config_YawKp, Config_YawKi, Config_YawKd);
        PID_Init(&pid_Vz, Config_VzKp, Config_VzKi, Config_VzKd);

        isConfigurationSet = 1;

        LogInformation(1001, "Configuration set successfully");
        ReplyPackage(ConfigurationSet);
    }
    else if (data[0] == 2)
    {
        if (data[1] == 1)
        {
            // Motors_Speed[0] = Config_BaseSpeed;
            // Motors_Speed[1] = Config_BaseSpeed;
            // Motors_Speed[2] = Config_BaseSpeed;
            // Motors_Speed[3] = Config_BaseSpeed;
            
            isDroneStarted = 1;
            LogInformation(1001, "DRONE STARTED");
            ReplyPackage(DroneStarted);
        }
        else if (data[1] == 2)
        {
            // Motors_Speed[0] = 0;
            // Motors_Speed[1] = 0;
            // Motors_Speed[2] = 0;
            // Motors_Speed[3] = 0;
            // Motors_SetSpeed(Motors_Speed);
            isDroneStarted = 0;
            LogInformation(1001, "DRONE STOPPED");
            ReplyPackage(DroneStopped);
        }
    }
}

void ReplyPackage(enum ResponseType responseType)
{
    uint8_t tx_data[32];
    memset(tx_data, '\0', sizeof(tx_data));

    switch (responseType)
    {
    case ConfigurationSet:
        tx_data[0] = 1;
        tx_data[1] = 1;

        break;
    case DroneStarted:
        tx_data[0] = 2;
        tx_data[1] = 1;

        break;
    case DroneStopped:
        tx_data[0] = 2;
        tx_data[1] = 2;
        break;
    }

    nrf24l01p_flush_tx_fifo();
    nrf24l01p_tx_irq();

    nrf24l01p_ptx_mode();
    nrf24l01p_tx_transmit(tx_data);

    uint16_t timer = HAL_GetTick();

    while (HAL_GetTick() - timer < 5)
    {
        uint8_t status = nrf24l01p_get_status();

        if (status & NRF24L01P_TX_DS || status & NRF24L01P_MAX_RT)
        {
            break;
        }
    }
    
    nrf24l01p_flush_tx_fifo();
    nrf24l01p_tx_irq();
    
    nrf24l01p_prx_mode();
}