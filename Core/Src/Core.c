#include "Core.h"
#include "Tools.h"
#include "MotorControl.h"
#include "string.h"
#include "NRF24L01P.h"
#include "MPU6050.h"
#include "BMPXX80.h"
#include "FlightControl.h"
#include "stdio.h"

// Variables
float altitude = 0.0f;
double angle_x = 0.0;
double angle_y = 0.0;
double angle_z = 0.0;
double vertical_velocity = 0.0;
double LEDTimer = 0.0;

// Static variables for internal use
static uint8_t USART_ReceivedByte;
static uint8_t USART_Buffer[500];
static uint16_t USART_dataIndex = 0;
uint8_t rx_data[32];

// State Manager
DroneStateManager_t droneState = {
    .state = DRONE_STATE_INIT,
    .lastLedUpdate = 0,
    .ledBlinkCounter = 0,
    .ledIsOn = 0
};

// Motor Variables
MotorSpeeds_t Motors_Speed = {
    .front_left = 0.0,
    .front_right = 0.0,
    .back_left = 0.0,
    .back_right = 0.0
};

// Config Variables
DroneConfig_t Config = {0};

// PID Variables
PID_t pid_roll = {0};
PID_t pid_pitch = {0};
PID_t pid_yaw = {0};
PID_t pid_Vz = {0};

uint8_t status = 0;

// Timing variables
static double last_update_time = 0.0;

void Core_init(void)
{
    // Init Logger
    HAL_UART_Receive_IT(&huart1, &USART_ReceivedByte, 1);

    // Init Motors
    Motors_Init(&htim1, &htim2, &htim3, &htim4);

    // Init MPU6050
    MPU6050_Init(&hi2c1);

    // Init BMP280
    BMP280_Init(&hi2c2, BMP280_TEMPERATURE_16BIT, BMP280_STANDARD, BMP280_FORCEDMODE);
    HAL_Delay(3000);

    // Enhanced NRF24L01 initialization
    nrf24l01p_init(2500, _250kbps);

    // Wait for radio to stabilize
    HAL_Delay(5);

    // Verify initialization
    if (!nrf24l01p_is_initialized())
    {
        LogError(2001, "NRF24L01+ initialization failed!");
        return;
    }

    uint8_t rx_address[5] = {0x11, 0x22, 0x33, 0x44, 0x55};

    nRF24L01_Set_Rx_Addr(rx_address, NRF24L01P_REG_RX_ADDR_P0);

    nrf24l01p_prx_mode();

    LogInformation(1001, "NRF24L01+ Started Successfully");

    // Getting ready to fly
    LogInformation(1002, "Doing Last Checks...");
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_Delay(2000);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    LogInformation(1001, "Drone is ready to Fly!");

    // Waiting for Configurations to be set
    LogInformation(1001, "Waiting for Configurations...");

    // Initialize flight control
    FlightControl_Init();
    droneState.lastLedUpdate = HAL_GetTick();
}

void Core_loop(void)
{
    uint32_t currentTime = HAL_GetTick();
    double dt = (currentTime - last_update_time) / 1000.0;
    last_update_time = currentTime;

    // Update LED based on current state
    UpdateLEDState();

    // Flight control logic
    if (droneState.state == DRONE_STATE_FLYING) {
        FlightControl_Update(dt);
    }

    status = nrf24l01p_get_status();

    // Check if data is available
    if (status & NRF24L01P_RX_DR)
    {
        uint8_t fifo_status;
        do
        {
            memset(rx_data, 0, sizeof(rx_data));
            nrf24l01p_read_rx_fifo(rx_data);
            HandlePackage(rx_data);
            fifo_status = nrf24l01p_get_fifo_status();
        } while (!(fifo_status & 0x01)); // Bit0 (RX_EMPTY) set means FIFO is empty

        nrf24l01p_clear_rx_dr();
    }

    HAL_Delay(10); // Small delay to avoid busy-waiting
}

void HandlePackage(uint8_t *data)
{
    switch(data[0]) {
        case 1: // Configuration
            Config.throttle = data[1]; 
            Config.minSpeed = data[2];
            Config.maxSpeed = data[3];
            Config.maxAngle = data[4];
            Config.pitch.target = data[5];
            Config.roll.target = data[6];
            Config.yaw.target = data[7];
            Config.vz.target = parse_scaled_float(data[8], 100.0f);

            // Parse PID values
            Config.pitch.Kp = parse_scaled_float(data[9], 10.0f);
            Config.pitch.Ki = parse_scaled_float_16bit(data[11], data[10], 1000.0f);
            Config.pitch.Kd = parse_scaled_float_16bit(data[13], data[12], 10000.0f);

            Config.roll.Kp = parse_scaled_float(data[15], 10.0f);
            Config.roll.Ki = parse_scaled_float_16bit(data[17], data[16], 1000.0f);
            Config.roll.Kd = parse_scaled_float_16bit(data[19], data[18], 10000.0f);

            Config.yaw.Kp = parse_scaled_float(data[20], 10.0f);
            Config.yaw.Ki = parse_scaled_float_16bit(data[22], data[21], 1000.0f);
            Config.yaw.Kd = parse_scaled_float_16bit(data[24], data[23], 10000.0f);

            Config.vz.Kp = parse_scaled_float(data[25], 10.0f);
            Config.vz.Ki = parse_scaled_float_16bit(data[27], data[26], 1000.0f);
            Config.vz.Kd = parse_scaled_float_16bit(data[29], data[28], 10000.0f);

            apply_pid_config();
            droneState.state = DRONE_STATE_CONFIGURED;
            LogInformation(1001, "Configuration set successfully");
            break;

        case 2: // Drone control
            if (droneState.state == DRONE_STATE_INIT) {
                LogError(2002, "Cannot start: Configuration not set");
                return;
            }
            
            if (droneState.state == DRONE_STATE_FLYING) {
                FlightControl_Stop();
                droneState.state = DRONE_STATE_CONFIGURED;
                Config.throttle = Config.minSpeed;
                LogInformation(1001, "DRONE STOPPED");
            } else {
                FlightControl_Start();
                droneState.state = DRONE_STATE_FLYING;
                last_update_time = HAL_GetTick();
                LogInformation(1001, "DRONE STARTED");
            }
            break;
    }
}

void apply_pid_config() {
    // Initialize PID controllers with new gains
    PID_Init(&pid_roll, Config.roll.Kp, Config.roll.Ki, Config.roll.Kd);
    PID_Init(&pid_pitch, Config.pitch.Kp, Config.pitch.Ki, Config.pitch.Kd);
    PID_Init(&pid_yaw, Config.yaw.Kp, Config.yaw.Ki, Config.yaw.Kd);
    PID_Init(&pid_Vz, Config.vz.Kp, Config.vz.Ki, Config.vz.Kd);
}

void UpdateLEDState(void) {
    uint32_t currentTime = HAL_GetTick();
    
    switch(droneState.state) {
        case DRONE_STATE_INIT:
            // Simple 1Hz blink
            if (currentTime - droneState.lastLedUpdate >= 1000) {
                HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
                droneState.lastLedUpdate = currentTime;
            }
            break;
            
        case DRONE_STATE_CONFIGURED:
            if (currentTime - droneState.lastLedUpdate >= (droneState.ledIsOn ? LED_SHORT_DELAY : 
                (droneState.ledBlinkCounter >= LED_CONFIG_BLINKS ? LED_LONG_DELAY : LED_SHORT_DELAY))) {
                
                if (droneState.ledBlinkCounter >= LED_CONFIG_BLINKS) {
                    droneState.ledBlinkCounter = 0;
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
                } else {
                    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
                    if (droneState.ledIsOn) droneState.ledBlinkCounter++;
                }
                droneState.ledIsOn = !droneState.ledIsOn;
                droneState.lastLedUpdate = currentTime;
            }
            break;
            
        case DRONE_STATE_FLYING:
            if (currentTime - droneState.lastLedUpdate >= (droneState.ledIsOn ? LED_SHORT_DELAY : 
                (droneState.ledBlinkCounter >= LED_STARTED_BLINKS ? LED_LONG_DELAY : LED_SHORT_DELAY))) {
                
                if (droneState.ledBlinkCounter >= LED_STARTED_BLINKS) {
                    droneState.ledBlinkCounter = 0;
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
                } else {
                    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
                    if (droneState.ledIsOn) droneState.ledBlinkCounter++;
                }
                droneState.ledIsOn = !droneState.ledIsOn;
                droneState.lastLedUpdate = currentTime;
            }
            break;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (USART_ReceivedByte == '\n')
        {
            memset(USART_Buffer, '\0', sizeof(char) * sizeof(USART_Buffer));
            USART_dataIndex = 0;
        }
        else
        {
            USART_Buffer[USART_dataIndex++] = USART_ReceivedByte;
        }

        HAL_UART_Receive_IT(&huart1, &USART_ReceivedByte, 1);
    }
}