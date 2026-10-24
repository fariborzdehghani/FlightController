#include "Core.h"
#include "AppError.h"
#include "BMPXX80.h"
#include "CoreHelpers.h"
#include "CommandProtocol.h"
#include "FlightControl.h"
#include "FlightRecorder.h"
#include "Logger.h"
#include "MPU6050.h"
#include "MotorControl.h"
#include "PID.h"
#include "QMC5883.h"
#include "SensorAcquisition.h"
#include "SensorConfig.h"
#include "Tools.h"
#include "srf05.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include "sx127x.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart1;
uint8_t uart_buffer[PACKET_LENGTH];
volatile uint8_t uart_data_ready = 0;
uint8_t uart_data_buffer[PACKET_LENGTH];

static const QMC5883P_Config_t qmc5883Config = {
    .i2c = &hi2c2,
    .address_7bit = QMC5883P_DEFAULT_ADDR_7BIT,
    .timeout_ms = 100,
    .mode = QMC_MODE_CONTINUOUS,
    .odr = QMC_ODR_50HZ,
    .osr1 = QMC_OSR1_4,
    .osr2 = QMC_OSR2_4,
    .range = QMC_RANGE_8G,
    .setreset = QMC_SETRESET_ON_AND_ON};

static const SRF05_Config_t srf05Config = {.timer = &htim3,
                                           .captureChannel = TIM_CHANNEL_3,
                                           .triggerPort = GPIOB,
                                           .triggerPin = GPIO_PIN_1,
                                           .timeout_ms = 60,
                                           .timer_tick_us = 1.0f};

static const BMP280_Config_t bmp280Config = {
    .i2c = &hi2c2,
    .temperature_resolution = BMP280_TEMPERATURE_20BIT,
    .pressure_oversampling = BMP280_ULTRAHIGHRES,
    .mode = BMP280_NORMALMODE,
    .standby_time = BME280_STANDBY_MS_0_5,
    .filter = BME280_FILTER_OFF};

// State Manager
DroneStateManager_t droneState = {.state = DRONE_STATE_INIT,
                                  .lastLedUpdate = 0,
                                  .ledBlinkCounter = 0,
                                  .ledIsOn = 0};

// Config Variables
DroneConfig_t Config = {0};
FlightData_t flightData = {0};

// Motor Variables
MotorSpeeds_t Motors_Speed = {
    .front_left = 0.0, .front_right = 0.0, .back_left = 0.0, .back_right = 0.0};

// PID Variables
PID_t pid_roll = {0};  // Initialize roll PID controller
PID_t pid_pitch = {0}; // Initialize pitch PID controller
PID_t pid_Gz = {0};    // Initialize Gz (angular velocity) PID controller
PID_t pid_altitude = {0}; // Initialize altitude PID controller

// Timing variables
static uint32_t last_update_time;
static uint32_t last_command_time;
static volatile double average_dt;
static uint32_t dt_sample_count;
static uint8_t hardware_ready;
static uint8_t remote_commands_enabled;
static uint8_t core_initialization_complete;
static uint8_t fault_recoverable;
static uint8_t watchdog_started;
static DroneState_t led_displayed_state = DRONE_STATE_INIT;
static uint8_t led_display_initialized;
static uint8_t led_displayed_command_ready;
static IWDG_HandleTypeDef hiwdg;
static uint8_t packet_from_uart;
static uint8_t log_response[64];
static uint8_t log_response_pending, log_response_uart, log_radio_tx_active;
static uint32_t log_response_queued_at;
static uint8_t led_displayed_transfer;

static uint8_t Core_MotorsActive(void)
{
  return droneState.state == DRONE_STATE_ARMED ||
         droneState.state == DRONE_STATE_TAKEOFF ||
         droneState.state == DRONE_STATE_FLYING;
}

static void Core_ProcessLogResponse(void)
{
  if (log_radio_tx_active)
  {
    if (SX127X_PollSend()) log_radio_tx_active = 0u;
    return;
  }
  if (!log_response_pending || Core_MotorsActive()) return;
  if (log_response_uart)
  {
    const char hex[] = "0123456789ABCDEF";
    char line[139] = "FlightLog=";
    for (unsigned i = 0; i < 64; i++)
    {
      line[10u + i * 2u] = hex[log_response[i] >> 4];
      line[11u + i * 2u] = hex[log_response[i] & 15u];
    }
    line[138] = '\0';
    if (LogStringToPC(line)) log_response_pending = 0u;
  }
  else if (HAL_GetTick() - log_response_queued_at >= 30u)
  {
    /* Give the bridge time to switch from transmitting to listening. */
    if (SX127X_StartSend(log_response, sizeof(log_response)))
      log_radio_tx_active = 1u;
    log_response_pending = 0u; /* The desktop retries if TX fails. */
  }
}

static HAL_StatusTypeDef Core_StartWatchdog(void)
{
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_128;
  // About eight seconds at the nominal 32 kHz LSI, leaving margin for
  // foreground I/O and fault-reporting paths.
  hiwdg.Init.Reload = 1999u;
  HAL_StatusTypeDef status = HAL_IWDG_Init(&hiwdg);
  if (status == HAL_OK)
  {
    watchdog_started = 1u;
  }
  return status;
}

static void Core_EnterFault(const char *message)
{
  fault_recoverable = core_initialization_complete;
  FlightControl_Stop();
  Logger_EndFlight(FLIGHT_LOG_STOP_FAULT, AppError_GetCode(),
                   AppError_GetHalStatus(), DRONE_STATE_FAULT);
  droneState.state = DRONE_STATE_FAULT;
  hardware_ready = 0u;
  LogError(2003, message);
}

static void Core_ResetRuntimeFault(void)
{
  const uint32_t reset_time = HAL_GetTick();

  FlightControl_Stop();
  AppError_Clear();

  average_dt = 0.0;
  dt_sample_count = 0U;
  last_command_time = reset_time;
  last_update_time = reset_time;

  droneState.lastLedUpdate = reset_time;
  droneState.ledBlinkCounter = 0U;
  droneState.ledIsOn = 0U;
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

  hardware_ready = 1U;
  fault_recoverable = 0U;
  droneState.state = DRONE_STATE_CONFIGURED;
}

void Core_init(void)
{
  core_initialization_complete = 0U;
  fault_recoverable = 0U;
  hardware_ready = 0U;
  remote_commands_enabled = 0U;
  average_dt = 0.0;
  dt_sample_count = 0U;

  /* Match the logical Motor1..Motor4 order to the labels in the IOC:
   * M1=TIM3, M2=TIM2, M3=TIM4, M4=TIM1. */
  if (Motors_Init(&htim3, &htim2, &htim4, &htim1) != HAL_OK)
  {
    Core_EnterFault("Motor initialization failed");
    return;
  }

  // Init SX127X
  if (!SX127X_Init(&hspi2))
  {
    Core_EnterFault("SX127X initialization failed");
    return;
  }

  if (SENSOR_SRF05_ENABLED)
  {
    // Initialize SRF05
    if (SRF05_Init(srf05Config) != HAL_OK)
    {
      Core_EnterFault("SRF05 initialization failed");
      return;
    }
  }
  else
  {
    LogInformation(1003, "SRF05 disabled by sensor configuration");
  }

  if (SENSOR_QMC5883_ENABLED)
  {
    // Initialize QMC5883 compass
    if (QMC5883P_Init(qmc5883Config) != HAL_OK)
    {
      Core_EnterFault("QMC5883 initialization failed");
      return;
    }
  }
  else
  {
    LogInformation(1003, "QMC5883 disabled by sensor configuration");
  }

  if (SENSOR_BMP280_ENABLED)
  {
    // Initialize BMP280
    if (BMP280_Init(bmp280Config) != HAL_OK)
    {
      Core_EnterFault("BMP280 initialization failed");
      return;
    }
  }
  else
  {
    LogInformation(1003, "BMP280 disabled by sensor configuration");
  }

  if (SENSOR_MPU6050_ENABLED)
  {
    // Initialize MPU6050
    if (MPU6050_Init(&hi2c1) != HAL_OK)
    {
      Core_EnterFault("MPU6050 initialization failed");
      return;
    }
  }
  else
  {
    LogInformation(1003, "MPU6050 disabled by sensor configuration");
  }

  /* Sensor acquisition belongs to the device runtime, not the armed flight
   * session. Calibrate and warm the estimators once at boot, then keep their
   * cached data current from Core_loop(). */
  SensorAcquisition_Init();
  const SensorAcquisitionResult_t sensor_start_result =
      SensorAcquisition_Start();
  if (AppError_SetSensorAcquisitionResult(sensor_start_result) != HAL_OK)
  {
    Core_EnterFault(AppError_GetMessage());
    return;
  }
  flightData = SensorAcquisition_GetFlightData();
  hardware_ready = 1U;

  // Start UART interrupt receive for PACKET_LENGTH byte packets
  if (HAL_UART_Receive_IT(&huart1, uart_buffer, PACKET_LENGTH) != HAL_OK)
  {
    Core_EnterFault("UART receive initialization failed");
    return;
  }

  last_command_time = HAL_GetTick();
  last_update_time = last_command_time;

  if (!HEARTBEAT_FAILSAFE_ENABLED)
  {
    LogError(2004, "WARNING: heartbeat failsafe is disabled");
  }

  if (Core_StartWatchdog() != HAL_OK)
  {
    Core_EnterFault("Watchdog initialization failed");
    return;
  }

  /* Discard packets that may have arrived while calibration/startup was
   * blocking so a stale ARM or configuration command cannot execute as soon
   * as the gate opens. */
  uart_data_ready = 0U;
  SX127X_ClearIrqFlags();
  core_initialization_complete = 1U;
  remote_commands_enabled = 1U;
}

void Core_loop(void)
{
  uint32_t currentTime = HAL_GetTick();
  double dt = (double)(currentTime - last_update_time) / 1000.0;
  last_update_time = currentTime;
  Logger_SetLoopTiming((float)(dt * 1000.0));

  if (dt_sample_count < UINT32_MAX)
  {
    ++dt_sample_count;
    average_dt += (dt - average_dt) / (double)dt_sample_count;
  }

  UpdateLEDState();

  if (hardware_ready)
  {
    const SensorAcquisitionResult_t sensor_result =
        SensorAcquisition_Update(dt);
    flightData = SensorAcquisition_GetFlightData();
    if (AppError_SetSensorAcquisitionResult(sensor_result) != HAL_OK)
    {
      Core_EnterFault(AppError_GetMessage());
    }
  }

  if (droneState.state == DRONE_STATE_ARMED ||
      droneState.state == DRONE_STATE_TAKEOFF ||
      droneState.state == DRONE_STATE_FLYING)
  {
    if (HEARTBEAT_FAILSAFE_ENABLED && !FlightControl_IsLanding() &&
        (currentTime - last_command_time) >= COMMAND_FAILSAFE_TIMEOUT_MS)
    {
      FlightControl_Stop();
      Logger_EndFlight(FLIGHT_LOG_STOP_LINK_TIMEOUT, 0u, HAL_OK,
                       DRONE_STATE_CONFIGURED);
      droneState.state = DRONE_STATE_CONFIGURED;
      LogError(2004, "Control link timeout: drone disarmed");
    }
    else
    {
      if (FlightControl_Update(dt) != HAL_OK)
      {
        Core_EnterFault(AppError_GetMessage());
      }
      else if (droneState.state == DRONE_STATE_TAKEOFF &&
               FlightControl_IsTakeoffComplete())
      {
        /* The altitude controller continues unchanged through this transition. */
        droneState.state = DRONE_STATE_FLYING;
        LogInformation(1001, "Takeoff settled: holding configured altitude");
      }
    }
  }

  uint8_t radio_buffer[PACKET_LENGTH];
  uint8_t bytes_received;

  if (!log_radio_tx_active &&
      SX127X_Receive(radio_buffer, sizeof(radio_buffer), &bytes_received))
  {
    if (bytes_received == PACKET_LENGTH)
    {
      packet_from_uart = 0u;
      Logger_SetLiveSerial(0u);
      HandlePackage(radio_buffer, bytes_received);
    }
  }

  if (uart_data_ready)
  {
    packet_from_uart = 1u;
    Logger_SetLiveSerial(1u);
    HandlePackage(uart_data_buffer, PACKET_LENGTH);
    uart_data_ready = 0;
  }

  Core_ProcessLogResponse();
  Tools_ProcessLogQueue();
  if (watchdog_started)
  {
    (void)HAL_IWDG_Refresh(&hiwdg);
  }
  HAL_Delay(10);
}

void HandlePackage(const uint8_t *data, uint8_t length)
{
  if (!CommandPacket_IsValidShape(data, length))
  {
    return;
  }

  /* Control commands stay locked out until startup succeeds. On a startup
   * fault, allow only validated, read-only log requests so its cause can be
   * downloaded over the radio. The log handler still rejects active motors. */
  if (!remote_commands_enabled &&
      !(droneState.state == DRONE_STATE_FAULT && length == PACKET_LENGTH &&
        FlightLog_IsRequestValid(data)))
  {
    return;
  }

  switch (data[0])
  {
  case PACKET_TYPE_LOG_REQUEST:
    if (length != 64u || Core_MotorsActive() || log_radio_tx_active) return;
    if (FlightLog_IsRequestValid(data) && data[1] == FLIGHT_LOG_BEGIN)
      Logger_PrepareDownload();
    if (FlightRecorder_Respond(data, log_response, HAL_GetTick(), false))
    {
      log_response_uart = packet_from_uart;
      log_response_pending = 1u;
      log_response_queued_at = HAL_GetTick();
    }
    break;

  case PACKET_TYPE_CONFIGURATION:
  {
    if (length < 41u)
    {
      LogError(2005, "Configuration packet is too short");
      return;
    }
    if (droneState.state == DRONE_STATE_ARMED ||
        droneState.state == DRONE_STATE_TAKEOFF ||
        droneState.state == DRONE_STATE_FLYING)
    {
      LogError(2005, "Disarm before changing configuration");
      return;
    }

    DroneConfig_t candidate = Config;
    candidate.armThrottle = data[1];
    candidate.minThrottle = data[2];
    candidate.maxThrottle = data[3];
    candidate.maxAngle = data[4];

    /* Byte 43 optionally carries the measured hover throttle. A zero keeps
     * older senders working by choosing a reasonable first estimate halfway
     * between armed idle and maximum throttle. */
    candidate.hoverThrottle =
        length >= 44u && data[43] != 0U
            ? (double)data[43]
            : candidate.armThrottle +
                  0.5 * (candidate.maxThrottle - candidate.armThrottle);

    // Parse Target values as signed 16-bit
    candidate.pitch.target = (int16_t)(data[5] | (data[6] << 8));
    candidate.roll.target = (int16_t)(data[7] | (data[8] << 8));
    candidate.yaw.target = (int16_t)(data[9] | (data[10] << 8));
    candidate.Gz.target = (int16_t)(data[11] | (data[12] << 8));
    const uint16_t takeoff_altitude_cm =
        (uint16_t)((uint16_t)data[13] | ((uint16_t)data[14] << 8));
    candidate.takeoffAltitudeCm =
        takeoff_altitude_cm == 0U
            ? TAKEOFF_ALTITUDE_DEFAULT_CM
            : (double)takeoff_altitude_cm;

    // Parse Pitch PID
    candidate.pitch.Kp = (double)data[15] / 100.0;
    candidate.pitch.Ki = (double)(data[16] | (data[17] << 8)) / 10000.0;
    candidate.pitch.Kd = (double)(data[18] | (data[19] << 8)) / 1000.0;

    // Parse Roll PID
    candidate.roll.Kp = (double)data[20] / 100.0;
    candidate.roll.Ki = (double)(data[21] | (data[22] << 8)) / 10000.0;
    candidate.roll.Kd = (double)(data[23] | (data[24] << 8)) / 1000.0;

    // Parse Gz PID
    candidate.Gz.Kp = (double)data[25] / 100.0;
    candidate.Gz.Ki = (double)(data[26] | (data[27] << 8)) / 1000.0;
    candidate.Gz.Kd = (double)(data[28] | (data[29] << 8)) / 10000.0;

    // Parse Altitude PID
    candidate.altitude.Kp = (double)data[30] / 100.0;
    candidate.altitude.Ki = (double)(data[31] | (data[32] << 8)) / 1000.0;
    candidate.altitude.Kd = (double)(data[33] | (data[34] << 8)) / 10000.0;

    // Parse PID common settings
    candidate.pidMaxIPart = (double)(data[36] | (data[37] << 8)) / 10.0;
    candidate.pidMaxOutput = (double)(data[38] | (data[39] << 8)) / 10.0;

    uint16_t takeoff_throttle_ramp_tenths = 0U;
    if (length >= 43u)
    {
      takeoff_throttle_ramp_tenths =
          (uint16_t)((uint16_t)data[41] | ((uint16_t)data[42] << 8));
    }
    candidate.takeoffThrottleRampPerSecond =
        takeoff_throttle_ramp_tenths == 0U
            ? TAKEOFF_THROTTLE_RAMP_DEFAULT_PER_SECOND
            : (double)takeoff_throttle_ramp_tenths / 10.0;

    if (!CoreHelpers_IsConfigValid(&candidate))
    {
      LogError(2005, "Configuration values are outside safe limits");
      return;
    }

    const uint8_t recovering_from_fault =
        droneState.state == DRONE_STATE_FAULT;
    if (recovering_from_fault && !fault_recoverable)
    {
      LogError(2005,
               "Initialization fault cannot be reset by configuration");
      return;
    }

    Config = candidate;
    Logger_SetConfiguration(data, length);

    if (recovering_from_fault)
    {
      Core_ResetRuntimeFault();
    }

    apply_pid_config();
    last_command_time = HAL_GetTick();

    if (recovering_from_fault)
    {
      LogInformation(1001,
                     "Configuration set: runtime fault cleared");
    }
    else if (droneState.state == DRONE_STATE_INIT)
    {
      if (hardware_ready)
      {
        droneState.state = DRONE_STATE_CONFIGURED;
        LogInformation(1001, "Initial configuration set successfully");
      }
    }
    else
    {
      LogInformation(1001, "Configuration updated");
    }
    break;
  }

  case PACKET_TYPE_CONTROL:
    if (length < 2u)
    {
      LogError(2005, "Control packet is too short");
      return;
    }
    last_command_time = HAL_GetTick();

    if (data[1] == CONTROL_COMMAND_DISARM)
    {
      /* A bridge link timeout must not interrupt an already bounded landing.
       * An explicit operator DISARM (reason zero) always stops immediately. */
      if (data[2] == CONTROL_DISARM_REASON_LINK_TIMEOUT && FlightControl_IsLanding())
      {
        LogError(2004, "Control link lost: continuing gradual landing");
        break;
      }
      FlightControl_Stop();
      Logger_EndFlight(length >= 3u && data[2] == CONTROL_DISARM_REASON_LINK_TIMEOUT
                           ? FLIGHT_LOG_STOP_LINK_TIMEOUT : FLIGHT_LOG_STOP_OPERATOR,
                       0u, HAL_OK,
                       DRONE_STATE_CONFIGURED);
      FlightRecorder_CancelTransfer();
      log_response_pending = 0u;
      if (droneState.state != DRONE_STATE_FAULT &&
          droneState.state != DRONE_STATE_INIT)
      {
        droneState.state = DRONE_STATE_CONFIGURED;
      }
      LogInformation(1001, "DRONE DISARMED");
      break;
    }

    if (data[1] == CONTROL_COMMAND_TAKEOFF)
    {
      if (droneState.state == DRONE_STATE_TAKEOFF ||
          droneState.state == DRONE_STATE_FLYING)
      {
        LogInformation(1001, "TAKEOFF is already active");
        break;
      }
      if (droneState.state != DRONE_STATE_ARMED)
      {
        LogError(2002, "ARM the drone before TAKEOFF");
        break;
      }

      const double takeoff_tilt_limit =
          fmin(Config.maxAngle, TAKEOFF_MAX_TILT_DEG);
      if (!isfinite(flightData.roll_deg) ||
          !isfinite(flightData.pitch_deg) ||
          fabs(flightData.roll_deg) > takeoff_tilt_limit ||
          fabs(flightData.pitch_deg) > takeoff_tilt_limit)
      {
        /* Fits the recorder's 96-byte message payload, including non-finite
         * readings. These are the same values used by the check above. */
        char message[97];
        snprintf(message, sizeof(message),
                 "TAKEOFF rejected: tilt; roll=%.6g pitch=%.6g limit=%.6g deg",
                 flightData.roll_deg, flightData.pitch_deg, takeoff_tilt_limit);
        LogError(2002, message);
        break;
      }

      if (flightData.takeoff_altitude_valid &&
          Config.takeoffAltitudeCm <
              flightData.takeoff_altitude + TAKEOFF_MIN_CLIMB_CM)
      {
        LogError(2002,
                 "TAKEOFF rejected: altitude target is too close");
        break;
      }

      if (SENSOR_MPU6050_ENABLED && SENSOR_SRF05_ENABLED)
      {
        FlightControl_BeginTakeoff();
        last_update_time = HAL_GetTick();
        droneState.state = DRONE_STATE_TAKEOFF;
        Logger_BeginFlight();
        LogInformation(
            1001,
            "TAKEOFF started: bounded ramp, then gradual altitude climb");
      }
      else
      {
        LogError(2002, "TAKEOFF requires MPU6050 and SRF05");
      }
      break;
    }

    if (data[1] != CONTROL_COMMAND_ARM)
    {
      LogError(2005, "Unknown control command");
      return;
    }

    if (FlightRecorder_IsTransferring(HAL_GetTick()) ||
        log_response_pending || log_radio_tx_active)
    {
      LogError(2002, "Finish or cancel log download before arming");
      return;
    }

    if (droneState.state == DRONE_STATE_INIT ||
        droneState.state == DRONE_STATE_FAULT || !hardware_ready ||
        !CoreHelpers_IsConfigValid(&Config))
    {
      LogError(2002, "Cannot start: valid configuration required");
      return;
    }

    if (droneState.state == DRONE_STATE_ARMED ||
        droneState.state == DRONE_STATE_TAKEOFF ||
        droneState.state == DRONE_STATE_FLYING)
    {
      break;
    }

    if (FlightControl_Start() == HAL_OK)
    {
      droneState.state = DRONE_STATE_ARMED;
      const uint32_t armed_time = HAL_GetTick();
      last_command_time = armed_time;
      last_update_time = armed_time;
      LogInformation(1001, "DRONE ARMED: motors at configured arm throttle");
    }
    else if (AppError_GetCode() == APP_ERROR_MPU6050_CALIBRATION)
    {
      /* Calibration failure is recoverable. Motors remain stopped and the
       * pilot can place the aircraft still before sending ARM again. */
      FlightControl_Stop();
      LogError(2002, AppError_GetMessage());
    }
    else
    {
      Core_EnterFault(AppError_GetMessage());
    }
    break;

  case PACKET_TYPE_HEARTBEAT:
    last_command_time = HAL_GetTick();
    break;

  default:
    LogError(2005, "Unknown packet type");
    break;
  }
}

void apply_pid_config()
{
  // Initialize PID controllers with new gains
  PID_Init(&pid_roll, Config.roll.Kp, Config.roll.Ki, Config.roll.Kd);
  PID_Init(&pid_pitch, Config.pitch.Kp, Config.pitch.Ki, Config.pitch.Kd);
  PID_Init(&pid_Gz, Config.Gz.Kp, Config.Gz.Ki, Config.Gz.Kd);
  PID_Init(&pid_altitude, Config.altitude.Kp, Config.altitude.Ki,
           Config.altitude.Kd);
}

static void UpdateLEDBlinkPattern(uint32_t currentTime, uint8_t blinkCount)
{
  if (droneState.ledIsOn)
  {
    if ((currentTime - droneState.lastLedUpdate) < LED_SHORT_DELAY)
    {
      return;
    }

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
    droneState.ledIsOn = 0U;
    droneState.ledBlinkCounter++;
    droneState.lastLedUpdate = currentTime;
    return;
  }

  const uint32_t offDelay =
      droneState.ledBlinkCounter >= blinkCount ? LED_LONG_DELAY
                                               : LED_SHORT_DELAY;
  if ((currentTime - droneState.lastLedUpdate) < offDelay)
  {
    return;
  }

  if (droneState.ledBlinkCounter >= blinkCount)
  {
    droneState.ledBlinkCounter = 0U;
  }

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
  droneState.ledIsOn = 1U;
  droneState.lastLedUpdate = currentTime;
}

void UpdateLEDState(void)
{
  uint32_t currentTime = HAL_GetTick();
  uint8_t transferring = FlightRecorder_IsTransferring(currentTime) ||
                         log_response_pending || log_radio_tx_active;

  if (!led_display_initialized ||
      droneState.state != led_displayed_state ||
      remote_commands_enabled != led_displayed_command_ready ||
      transferring != led_displayed_transfer)
  {
    led_display_initialized = 1U;
    led_displayed_state = droneState.state;
    led_displayed_command_ready = remote_commands_enabled;
    led_displayed_transfer = transferring;
    droneState.lastLedUpdate = currentTime;
    droneState.ledBlinkCounter = 0U;
    droneState.ledIsOn = droneState.state == DRONE_STATE_INIT ? 1U : 0U;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,
                      droneState.ledIsOn ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }

  if (transferring)
  {
    /* Distinct double flash: 80 ms on, 80 off, 80 on, then 760 off. */
    uint32_t phase = (currentTime - droneState.lastLedUpdate) % 1000u;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,
        (phase < 80u || (phase >= 160u && phase < 240u)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    return;
  }

  switch (droneState.state)
  {
  case DRONE_STATE_INIT:
    if (remote_commands_enabled)
    {
      // Five short blinks: calibration complete and commands are accepted.
      UpdateLEDBlinkPattern(currentTime, LED_READY_BLINKS);
    }
    else if (currentTime - droneState.lastLedUpdate >= LED_LONG_DELAY)
    {
      // Startup/calibration still in progress: one second on/off.
      droneState.ledIsOn = !droneState.ledIsOn;
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,
                        droneState.ledIsOn ? GPIO_PIN_SET : GPIO_PIN_RESET);
      droneState.lastLedUpdate = currentTime;
    }
    break;

  case DRONE_STATE_CONFIGURED:
    UpdateLEDBlinkPattern(currentTime, LED_CONFIG_BLINKS);
    break;

  case DRONE_STATE_ARMED:
    UpdateLEDBlinkPattern(currentTime, LED_ARMED_BLINKS);
    break;

  case DRONE_STATE_TAKEOFF:
  case DRONE_STATE_FLYING:
    UpdateLEDBlinkPattern(currentTime, LED_FLIGHT_BLINKS);
    break;

  case DRONE_STATE_FAULT:
    if (currentTime - droneState.lastLedUpdate >= 100u)
    {
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
      droneState.lastLedUpdate = currentTime;
    }
    break;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    if (!uart_data_ready)
    {
      memcpy(uart_data_buffer, uart_buffer, PACKET_LENGTH);
      uart_data_ready = 1;
    }
    // Restart receive for next packet
    (void)HAL_UART_Receive_IT(&huart1, uart_buffer, PACKET_LENGTH);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  Tools_UART_TxCpltCallback(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  Tools_UART_ErrorCallback(huart);
  if (huart != NULL && huart->Instance == USART1)
  {
    (void)HAL_UART_Receive_IT(&huart1, uart_buffer, PACKET_LENGTH);
  }
}
