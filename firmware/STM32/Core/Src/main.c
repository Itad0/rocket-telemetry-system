/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Secure Rocket Telemetry System - STM32 Firmware
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;  // GPS
UART_HandleTypeDef huart2;  // Debug
SPI_HandleTypeDef hspi1;    // LoRa
I2C_HandleTypeDef hi2c1;    // Sensors
TIM_HandleTypeDef htim2;    // System timer

/* Flight State Machine */
typedef enum {
    STATE_BOOT = 0,
    STATE_CALIBRATION,
    STATE_LAUNCH,
    STATE_ASCENT,
    STATE_APOGEE,
    STATE_DESCENT,
    STATE_LANDING
} FlightState_t;

/* Telemetry Data Structure */
typedef struct {
    uint32_t timestamp;
    FlightState_t state;
    float altitude;
    float temperature;
    float pressure;
    float humidity;
    float voltage;
    float current;
    struct {
        float latitude;
        float longitude;
        uint8_t satellites;
    } gps;
    struct {
        float roll;
        float pitch;
        float yaw;
    } orientation;
} TelemetryData_t;

/* Global Variables */
static FlightState_t flight_state = STATE_BOOT;
static TelemetryData_t telemetry_data = {0};
static uint8_t aes_key[16] = {0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
                              0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c};

/* Function Prototypes -------------------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);

static void InitializeSensors(void);
static void ReadSensors(void);
static void TransmitTelemetry(void);
static void UpdateFlightState(void);
static void EncryptData(uint8_t *data, size_t len);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* MCU Configuration--------------------------------------------------------*/
    HAL_Init();
    SystemClock_Config();
    
    /* Initialize peripherals */
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_SPI1_Init();
    MX_I2C1_Init();
    MX_TIM2_Init();
    
    /* Start timer */
    HAL_TIM_Base_Start_IT(&htim2);
    
    /* Initialize sensors */
    InitializeSensors();
    
    /* Boot message */
    char msg[64];
    sprintf(msg, "Rocket Telemetry System v1.0 - BOOT\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
    
    /* Main loop */
    while (1)
    {
        /* Read all sensors */
        ReadSensors();
        
        /* Update flight state machine */
        UpdateFlightState();
        
        /* Transmit encrypted telemetry */
        TransmitTelemetry();
        
        /* 100ms delay for 10Hz update rate */
        HAL_Delay(100);
    }
}

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    
    /* Configure the main internal regulator output voltage */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    
    /* Initializes the RCC Oscillators */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    
    /* Initializes the CPU, AHB and APB buses clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief Initialize all sensors
  */
static void InitializeSensors(void)
{
    /* MS5611 Barometer Init */
    uint8_t ms5611_reset = 0x1E;
    HAL_I2C_Master_Transmit(&hi2c1, 0x77 << 1, &ms5611_reset, 1, 100);
    HAL_Delay(10);
    
    /* BNO055 IMU Init */
    uint8_t bno055_config[] = {0x3D, 0x0C};  // NDOF mode
    HAL_I2C_Master_Transmit(&hi2c1, 0x28 << 1, bno055_config, 2, 100);
    HAL_Delay(20);
    
    /* Initialize LoRa module */
    // LoRa initialization would go here
    
    flight_state = STATE_CALIBRATION;
}

/**
  * @brief Read all sensor values
  */
static void ReadSensors(void)
{
    uint8_t buffer[8];
    
    /* Update timestamp */
    telemetry_data.timestamp = HAL_GetTick();
    
    /* Read MS5611 (pressure/altitude) */
    // Simplified - actual implementation would follow MS5611 protocol
    HAL_I2C_Master_Transmit(&hi2c1, 0x77 << 1, (uint8_t[]){0x48}, 1, 100);
    HAL_Delay(10);
    HAL_I2C_Master_Receive(&hi2c1, 0x77 << 1, buffer, 3, 100);
    
    uint32_t pressure_raw = (buffer[0] << 16) | (buffer[1] << 8) | buffer[2];
    telemetry_data.pressure = pressure_raw / 100.0f;  // Simplified conversion
    telemetry_data.altitude = 44330.0f * (1.0f - powf(telemetry_data.pressure / 1013.25f, 0.1903f));
    
    /* Read BNO055 (orientation) */
    uint8_t reg = 0x1A;  // Euler angles register
    HAL_I2C_Master_Transmit(&hi2c1, 0x28 << 1, &reg, 1, 100);
    HAL_I2C_Master_Receive(&hi2c1, 0x28 << 1, buffer, 6, 100);
    
    telemetry_data.orientation.roll = ((int16_t)(buffer[1] << 8 | buffer[0])) / 16.0f;
    telemetry_data.orientation.pitch = ((int16_t)(buffer[3] << 8 | buffer[2])) / 16.0f;
    telemetry_data.orientation.yaw = ((int16_t)(buffer[5] << 8 | buffer[4])) / 16.0f;
    
    /* Simulate other sensor readings for demo */
    telemetry_data.temperature = 20.0f + (rand() % 100) / 10.0f;
    telemetry_data.humidity = 45.0f + (rand() % 200) / 10.0f;
    telemetry_data.voltage = 3.7f + (rand() % 10) / 100.0f;
    telemetry_data.current = 0.5f + (rand() % 50) / 100.0f;
}

/**
  * @brief Update flight state based on sensor data
  */
static void UpdateFlightState(void)
{
    static float last_altitude = 0;
    static uint32_t state_timer = 0;
    
    float velocity = (telemetry_data.altitude - last_altitude) * 10.0f;  // m/s (10Hz update)
    last_altitude = telemetry_data.altitude;
    
    switch (flight_state)
    {
        case STATE_CALIBRATION:
            if (HAL_GetTick() - state_timer > 5000)  // 5 seconds calibration
            {
                flight_state = STATE_LAUNCH;
                state_timer = HAL_GetTick();
            }
            break;
            
        case STATE_LAUNCH:
            if (velocity > 5.0f)  // Launch detected
            {
                flight_state = STATE_ASCENT;
                state_timer = HAL_GetTick();
            }
            break;
            
        case STATE_ASCENT:
            if (velocity < 0.5f)  // Near apogee
            {
                flight_state = STATE_APOGEE;
                state_timer = HAL_GetTick();
            }
            break;
            
        case STATE_APOGEE:
            if (velocity < -2.0f)  // Descending
            {
                flight_state = STATE_DESCENT;
                state_timer = HAL_GetTick();
            }
            break;
            
        case STATE_DESCENT:
            if (telemetry_data.altitude < 100.0f && velocity > -1.0f)
            {
                flight_state = STATE_LANDING;
                state_timer = HAL_GetTick();
            }
            break;
            
        case STATE_LANDING:
            // Mission complete
            break;
            
        default:
            break;
    }
    
    telemetry_data.state = flight_state;
}

/**
  * @brief Transmit encrypted telemetry via LoRa
  */
static void TransmitTelemetry(void)
{
    uint8_t packet[128];
    size_t packet_size = 0;
    
    /* Create telemetry packet */
    memcpy(packet, &telemetry_data, sizeof(TelemetryData_t));
    packet_size = sizeof(TelemetryData_t);
    
    /* Encrypt packet */
    EncryptData(packet, packet_size);
    
    /* Transmit via LoRa (simplified - actual implementation would use LoRa protocol) */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);  // CS Low
    HAL_SPI_Transmit(&hspi1, packet, packet_size, 100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);    // CS High
    
    /* Debug output */
    char msg[64];
    sprintf(msg, "TX: Alt=%.1fm State=%d\r\n", telemetry_data.altitude, flight_state);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
}

/**
  * @brief Simple AES-128 encryption (placeholder)
  */
static void EncryptData(uint8_t *data, size_t len)
{
    /* In production, use proper AES-128 implementation */
    for (size_t i = 0; i < len; i++)
    {
        data[i] ^= aes_key[i % 16];  // Simple XOR for demo
    }
}

/**
  * @brief GPIO Initialization
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
    /* Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);  // LoRa CS
    
    /* Configure GPIO pins */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* Peripheral initialization functions would go here */
static void MX_USART1_UART_Init(void) { /* GPS UART */ }
static void MX_USART2_UART_Init(void) { /* Debug UART */ }
static void MX_SPI1_Init(void) { /* LoRa SPI */ }
static void MX_I2C1_Init(void) { /* Sensor I2C */ }
static void MX_TIM2_Init(void) { /* System Timer */ }

/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);  // LED blink
        HAL_Delay(100);
    }
}