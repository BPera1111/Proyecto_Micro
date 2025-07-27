/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <stdint.h>
#include "usbd_cdc_if.h"
#include "stm32f1xx_hal.h"
#include "gcode_parser.h"
//#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBUG_MESSAGES 0  // Cambiar a 0 para desactivar mensajes de debug

// Configuración real de la máquina (valores calibrados)
#define STEPS_PER_MM_X 79      // Pasos por mm para eje X
#define STEPS_PER_MM_Y 79      // Pasos por mm para eje Y
#define STEPS_PER_MM_Z 3930    // Pasos por mm para eje Z

// Definiciones de segmentos para arcos
#define SEGMENTS 50
#define PI 3.14159265358979323846

// Método de transmisión USB CDC (puedes cambiar esto según tus necesidades)
typedef enum {
    USB_METHOD_DIRECT,      // Envío directo (puede perderse si está ocupado)
    USB_METHOD_RETRY,       // Envío con reintentos (bloquea hasta enviar)
    USB_METHOD_QUEUED       // Envío mediante cola (recomendado)
} USBTransmitMethod_t;

#define USB_TRANSMIT_METHOD USB_METHOD_QUEUED  // ← Cambiar aquí el método

// Función auxiliar para envío USB según el método configurado
void sendUSBText(const char* message) {
    uint16_t len = strlen(message);
    CDC_Transmit_Queued((uint8_t*)message, len);
    
    // switch (USB_TRANSMIT_METHOD) {
    //     case USB_METHOD_DIRECT:
    //         sendUSBText((uint8_t*)message);
    //         break;
            
    //     case USB_METHOD_RETRY:
    //         sendUSBText_WithRetry((uint8_t*)message, len, 3, 10);  // 3 reintentos, 10ms
    //         break;
            
    //     case USB_METHOD_QUEUED:
    //         if (!CDC_Transmit_Queued((uint8_t*)message, len)) {
    //             // Si la cola está llena, usar método directo como fallback
    //             sendUSBText((uint8_t*)message);
    //         }
    //         break;
    // }
}


// Códigos de alarma específicos de la máquina
#define ALARM_HARD_LIMIT        1   // Hard limit activo
#define ALARM_SOFT_LIMIT        2   // Soft limit activo
#define ALARM_ABORT_CYCLE       3   // Reset durante ciclo
#define ALARM_PROBE_FAIL_INITIAL 4  // Probe fail al inicio
#define ALARM_PROBE_FAIL_CONTACT 5  // Probe fail en contacto
#define ALARM_HOMING_FAIL_RESET  6  // Homing fail reset
#define ALARM_HOMING_FAIL_DOOR   7  // Homing fail puerta
#define ALARM_HOMING_FAIL_PULLOFF 8 // Homing fail pulloff
#define ALARM_HOMING_FAIL_APPROACH 9 // Homing fail approach
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
extern UART_HandleTypeDef huart1;
// Declarar buffer global
static char outputBuffer[200];
int bufferIndex = 0;
bool commandComplete = false;
int32_t currentX, currentY, currentZ;

// Variables para control de velocidad y feed rate
float currentFeedRate = 100.0;     // Feed rate en mm/min (valor por defecto)
float rapidRate = 1000.0;          // Velocidad rápida para G0 en mm/min
float maxFeedRate = 2000.0;        // Velocidad máxima permitida

// Buffer para recibir comandos por USB CDC
char usbBuffer[100];
int usbBufferIndex = 0;
bool usbCommandComplete = false;  // Cambiar a false para evitar procesamiento inicial

// Sistema de almacenamiento de programa G-code
#define MAX_GCODE_LINES 100        // Máximo número de líneas de G-code a almacenar
#define MAX_LINE_LENGTH 80         // Longitud máxima de cada línea de G-code
char gcodeProgram[MAX_GCODE_LINES][MAX_LINE_LENGTH];  // Buffer para almacenar el programa
int programLineCount = 0;          // Número de líneas actualmente almacenadas
int currentExecutingLine = 0;      // Línea que se está ejecutando actualmente
bool isStoringProgram = false;     // Flag para indicar si estamos en modo almacenamiento
bool isProgramLoaded = false;      // Flag para indicar si hay un programa cargado
bool isProgramRunning = false;     // Flag para indicar si el programa se está ejecutando

// Definiciones de pines equivalentes a Arduino
#define X_STEP_PIN    GPIO_PIN_6
#define X_DIR_PIN     GPIO_PIN_7
#define X_EN_PIN      GPIO_PIN_8
#define X_MIN_PIN     GPIO_PIN_12

#define Y_STEP_PIN    GPIO_PIN_9
#define Y_DIR_PIN     GPIO_PIN_3
#define Y_EN_PIN      GPIO_PIN_4
#define Y_MIN_PIN     GPIO_PIN_13

#define Z_STEP_PIN    GPIO_PIN_8
#define Z_DIR_PIN     GPIO_PIN_9
#define Z_EN_PIN      GPIO_PIN_10
#define Z_MIN_PIN     GPIO_PIN_14

#define LED_HORARIO     GPIO_PIN_0
#define LED_ANTIHORARIO GPIO_PIN_1

const uint16_t STEP_DELAY_US = 800;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
//static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void loop(void);
void setup(void);
void delay_us(uint32_t us);
// void moveAxes(float x, float y, float z);
void processGcode(const char* command);
void X_move(int32_t steps, bool dir);
void Y_move(int32_t steps, bool dir);
void Z_move(int32_t steps, bool dir);
void X_stepOnce(void);
void Y_stepOnce(void);
void Z_stepOnce(void);
void performHoming(void);
bool isEndstopPressed(char axis);
void enableSteppers(void);
void disableSteppers(void);
void showConfiguration(void);
void report_status_message(uint8_t status_code);

// Funciones para control de feed rate
uint32_t calculateStepDelay(float feedRate, float distance_mm);
void moveAxesWithFeedRate(float x, float y, float z, float feedRate, bool isRapid);
void arc_move_r(float x_end, float y_end, float r, int clockwise);  // Movimiento de arco con radio

// Funciones para manejo de programa G-code
void startProgramStorage(void);
void stopProgramStorage(void);
bool addLineToProgram(const char* line);
void clearProgram(void);
void runProgram(void);
void runNextLine(void);
void pauseProgram(void);
void stopProgram(void);
void showHelp(void);
void showQueueStatus(void);  // Nueva función de diagnóstico
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();  // Inicia USB CDC

  // Inicializar cola de transmisión USB CDC
  CDC_TxQueue_Init();

  // Inicialización similar al setup() de Arduino
  setup();

  // Envío inicial usando cola
  CDC_Transmit_Queued((uint8_t*)"G-code listo\r\n", 14); 
  
  #if DEBUG_MESSAGES
  // Mensaje adicional de debug
  CDC_Transmit_Queued((uint8_t*)"Sistema iniciado - Esperando comandos...\r\n", 42);
  #endif

  while (1)
  {
    // Procesar cola de transmisión USB CDC
    CDC_TxQueue_Process();
    
    // Equivalente al loop() de Arduino
    loop();
    
    // Pausa optimizada para reducir carga del procesador y terminal
    HAL_Delay(50);  // 50ms = 20Hz, reduce carga significativamente
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void){
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_USART2_UART_Init(void)
//{
//
//  /* USER CODE BEGIN USART2_Init 0 */
//
//  /* USER CODE END USART2_Init 0 */
//
//  /* USER CODE BEGIN USART2_Init 1 */
//
//  /* USER CODE END USART2_Init 1 */
//  huart2.Instance = USART2;
//  huart2.Init.BaudRate = 9600;
//  huart2.Init.WordLength = UART_WORDLENGTH_8B;
//  huart2.Init.StopBits = UART_STOPBITS_1;
//  huart2.Init.Parity = UART_PARITY_NONE;
//  huart2.Init.Mode = UART_MODE_TX_RX;
//  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
//  if (HAL_UART_Init(&huart2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN USART2_Init 2 */
//
//  /* USER CODE END USART2_Init 2 */
//
//}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB3 PB4 PB6
                           PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void delay_us(uint32_t us) {
    uint32_t cycles = (SystemCoreClock / 1000000L) * us;
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles);
}

void X_stepOnce(void) {
    HAL_GPIO_WritePin(GPIOB, X_STEP_PIN, GPIO_PIN_SET);
    delay_us(2);
    HAL_GPIO_WritePin(GPIOB, X_STEP_PIN, GPIO_PIN_RESET);
}

void Y_stepOnce(void) {
    HAL_GPIO_WritePin(GPIOB, Y_STEP_PIN, GPIO_PIN_SET);
    delay_us(2);
    HAL_GPIO_WritePin(GPIOB, Y_STEP_PIN, GPIO_PIN_RESET);
}

void Z_stepOnce(void) {
    HAL_GPIO_WritePin(GPIOA, Z_STEP_PIN, GPIO_PIN_SET);
    delay_us(2);
    HAL_GPIO_WritePin(GPIOA, Z_STEP_PIN, GPIO_PIN_RESET);
}

void X_move(int32_t steps, bool dir) {
    // Configura dirección
    HAL_GPIO_WritePin(GPIOB, X_DIR_PIN, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);

    // Enciende el LED correspondiente al sentido
    if (dir) {
        HAL_GPIO_WritePin(GPIOB, LED_HORARIO, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, LED_ANTIHORARIO, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOB, LED_HORARIO, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, LED_ANTIHORARIO, GPIO_PIN_SET);
    }

    // Ejecuta los pasos
    for (int32_t i = 0; i < steps; i++) {
        X_stepOnce();
        delay_us(STEP_DELAY_US);
    }

    // Apaga ambos LEDs al terminar
    HAL_GPIO_WritePin(GPIOB, LED_HORARIO, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, LED_ANTIHORARIO, GPIO_PIN_RESET);
}

void Y_move(int32_t steps, bool dir) {
    // Configura dirección
    HAL_GPIO_WritePin(GPIOB, Y_DIR_PIN, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);

    // Enciende el LED correspondiente al sentido
    if (dir) {
        HAL_GPIO_WritePin(GPIOB, LED_HORARIO, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, LED_ANTIHORARIO, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOB, LED_HORARIO, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, LED_ANTIHORARIO, GPIO_PIN_SET);
    }

    // Ejecuta los pasos
    for (int32_t i = 0; i < steps; i++) {
        Y_stepOnce();
        delay_us(STEP_DELAY_US);
    }

    // Apaga ambos LEDs al terminar
    HAL_GPIO_WritePin(GPIOB, LED_HORARIO, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, LED_ANTIHORARIO, GPIO_PIN_RESET);
}

void Z_move(int32_t steps, bool dir) {
    // Configura dirección
    HAL_GPIO_WritePin(GPIOA, Z_DIR_PIN, dir ? GPIO_PIN_RESET : GPIO_PIN_SET);

    // Enciende el LED correspondiente al sentido
    if (dir) {
        HAL_GPIO_WritePin(GPIOB, LED_HORARIO, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, LED_ANTIHORARIO, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOB, LED_HORARIO, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, LED_ANTIHORARIO, GPIO_PIN_SET);
    }

    // Ejecuta los pasos
    for (int32_t i = 0; i < steps; i++) {
        Z_stepOnce();
        delay_us(STEP_DELAY_US);
    }

    // Apaga ambos LEDs al terminar
    HAL_GPIO_WritePin(GPIOB, LED_HORARIO, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, LED_ANTIHORARIO, GPIO_PIN_RESET);
}


/**
  * @brief  Calcula el delay entre pasos basado en el feed rate
  * @param  feedRate: Velocidad en mm/min
  * @param  distance_mm: Distancia total del movimiento en mm
  * @retval Delay en microsegundos entre pasos
  */
uint32_t calculateStepDelay(float feedRate, float distance_mm) {
    if (feedRate <= 0) return STEP_DELAY_US; // Usar delay por defecto si es inválido
    
    // Calcular pasos por segundo para el eje dominante
    // feedRate está en mm/min, convertir a mm/s
    float feedRate_mm_per_sec = feedRate / 60.0;
    
    // Usar el eje con mayor resolución (Z) para el cálculo más conservador
    float steps_per_mm = STEPS_PER_MM_Z; // El más alto: 3930 steps/mm
    
    // Calcular pasos por segundo
    float steps_per_sec = feedRate_mm_per_sec * steps_per_mm;
    
    // Calcular delay en microsegundos entre pasos
    if (steps_per_sec <= 0) return STEP_DELAY_US;
    
    uint32_t delay_us = (uint32_t)(1000000.0 / steps_per_sec);
    
    // Limitar delay mínimo para evitar problemas de timing
    if (delay_us < 200) delay_us = 200; // Mínimo 200us = 5000 pasos/segundo máximo
    
    return delay_us;
}

/**
  * @brief  Movimiento de ejes con control de feed rate (versión avanzada)
  * @param  x, y, z: Coordenadas objetivo en mm
  * @param  feedRate: Velocidad en mm/min
  * @param  isRapid: true para G0 (rapid), false para G1 (linear)
  * @retval None
  */
void moveAxesWithFeedRate(float x, float y, float z, float feedRate, bool isRapid) {
    // Calcular posiciones objetivo en pasos
    int32_t targetX = !isnan(x) ? (int32_t)(x * STEPS_PER_MM_X) : currentX;
    int32_t targetY = !isnan(y) ? (int32_t)(y * STEPS_PER_MM_Y) : currentY;
    int32_t targetZ = !isnan(z) ? (int32_t)(z * STEPS_PER_MM_Z) : currentZ;
    
    // Calcular diferencias (pasos relativos)
    int32_t deltaX = targetX - currentX;
    int32_t deltaY = targetY - currentY;
    int32_t deltaZ = targetZ - currentZ;
    
    // Calcular distancia total en mm para determinar velocidad
    float distance_X = !isnan(x) ? fabs(x - (currentX / (float)STEPS_PER_MM_X)) : 0;
    float distance_Y = !isnan(y) ? fabs(y - (currentY / (float)STEPS_PER_MM_Y)) : 0;
    float distance_Z = !isnan(z) ? fabs(z - (currentZ / (float)STEPS_PER_MM_Z)) : 0;
    float total_distance = sqrt(distance_X*distance_X + distance_Y*distance_Y + distance_Z*distance_Z);
    
    // Seleccionar velocidad según el tipo de movimiento
    float effective_feedrate = isRapid ? rapidRate : feedRate;
    
    // Limitar velocidad máxima
    if (effective_feedrate > maxFeedRate) {
        effective_feedrate = maxFeedRate;
    }
    
    // Determinar direcciones
    bool dirX = (deltaX >= 0);
    bool dirY = (deltaY >= 0);
    bool dirZ = (deltaZ >= 0);
    
    // Configurar direcciones de los motores
    if (deltaX != 0) HAL_GPIO_WritePin(GPIOB, X_DIR_PIN, dirX ? GPIO_PIN_SET : GPIO_PIN_RESET);
    if (deltaY != 0) HAL_GPIO_WritePin(GPIOB, Y_DIR_PIN, dirY ? GPIO_PIN_SET : GPIO_PIN_RESET);
    if (deltaZ != 0) HAL_GPIO_WritePin(GPIOA, Z_DIR_PIN, dirZ ? GPIO_PIN_RESET : GPIO_PIN_SET);
    
    // Convertir a valores absolutos para el algoritmo
    deltaX = abs(deltaX);
    deltaY = abs(deltaY);
    deltaZ = abs(deltaZ);
    
    // Calcular delay basado en feed rate
    uint32_t step_delay = calculateStepDelay(effective_feedrate, total_distance);
    
    // Debug: verificar valores antes del mensaje
    #if DEBUG_MESSAGES
    snprintf(outputBuffer, sizeof(outputBuffer), "[DEBUG] x=%.2f y=%.2f z=%.2f rate=%.1f dist=%.2f\r\n", 
             x, y, z, effective_feedrate, total_distance);
    sendUSBText(outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));
    #endif
    
    // Mostrar información del movimiento con validación de valores
    float display_x = !isnan(x) ? x : (currentX / (float)STEPS_PER_MM_X);
    float display_y = !isnan(y) ? y : (currentY / (float)STEPS_PER_MM_Y);
    float display_z = !isnan(z) ? z : (currentZ / (float)STEPS_PER_MM_Z);
    
    // Convertir floats a enteros para evitar problemas de printf con floats
    int x_int = (int)display_x;
    int x_dec = (int)((display_x - x_int) * 100);
    int y_int = (int)display_y;
    int y_dec = (int)((display_y - y_int) * 100);
    int z_int = (int)display_z;
    int z_dec = (int)((display_z - z_int) * 100);
    int f_int = (int)effective_feedrate;
    int f_dec = (int)((effective_feedrate - f_int) * 10);
    int d_int = (int)total_distance;
    int d_dec = (int)((total_distance - d_int) * 100);

    snprintf(outputBuffer, sizeof(outputBuffer), "%s: X=%d.%02d Y=%d.%02d Z=%d.%02d F=%d.%d D=%d.%02dmm T=%lduS\r\n", 
           isRapid ? "G0 RAPID" : "G1 LINEAR",
           x_int, abs(x_dec),
           y_int, abs(y_dec), 
           z_int, abs(z_dec),
           f_int, abs(f_dec), 
           d_int, abs(d_dec), 
           (unsigned long)step_delay);
    sendUSBText(outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));
    // Algoritmo de interpolación lineal 3D (Bresenham modificado)
    int32_t maxSteps = deltaX;
    if (deltaY > maxSteps) maxSteps = deltaY;
    if (deltaZ > maxSteps) maxSteps = deltaZ;
    
    if (maxSteps == 0) return; // No hay movimiento
    
    // Variables para el algoritmo de Bresenham 3D
    int32_t errorX = maxSteps / 2;
    int32_t errorY = maxSteps / 2;
    int32_t errorZ = maxSteps / 2;
    
    // Encender LED indicador de movimiento
    HAL_GPIO_WritePin(GPIOB, isRapid ? LED_ANTIHORARIO : LED_HORARIO, GPIO_PIN_SET);
    
    // Ejecutar pasos interpolados con feed rate controlado
    for (int32_t step = 0; step < maxSteps; step++) {
        bool stepX = false, stepY = false, stepZ = false;
        
        // Algoritmo de Bresenham para X
        errorX += deltaX;
        if (errorX >= maxSteps) {
            errorX -= maxSteps;
            stepX = true;
        }
        
        // Algoritmo de Bresenham para Y
        errorY += deltaY;
        if (errorY >= maxSteps) {
            errorY -= maxSteps;
            stepY = true;
        }
        
        // Algoritmo de Bresenham para Z
        errorZ += deltaZ;
        if (errorZ >= maxSteps) {
            errorZ -= maxSteps;
            stepZ = true;
        }
        
        // Ejecutar pasos simultáneamente
        if (stepX) X_stepOnce();
        if (stepY) Y_stepOnce();
        if (stepZ) Z_stepOnce();
        
        // Delay controlado por feed rate
        delay_us(step_delay);
    }
    
    // Apagar LEDs
    HAL_GPIO_WritePin(GPIOB, LED_HORARIO, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, LED_ANTIHORARIO, GPIO_PIN_RESET);
    
    // Actualizar posiciones actuales
    currentX = targetX;
    currentY = targetY;
    currentZ = targetZ;
}



void arc_move_r(float x_end, float y_end, float r, int clockwise) {
    float x0 = currentX;
    float y0 = currentY;
    float x1 = x_end * STEPS_PER_MM_X;
    float y1 = y_end * STEPS_PER_MM_Y;
    r = r * STEPS_PER_MM_X; // Convertir radio a pasos

    float dx = x1 - x0;
    float dy = y1 - y0;
    float d = sqrt(dx * dx + dy * dy);

    if (d > 2 * fabs(r)) {
        // printf("Error: el radio es muy pequeño para unir los puntos.\n");
        sendUSBText("Error: el radio es muy pequeño para unir los puntos.\r\n");
        return;
    }

    // Punto medio entre inicio y fin
    float mx = (x0 + x1) / 2;
    float my = (y0 + y1) / 2;

    // Altura desde el punto medio al centro
    float h = sqrt(r * r - (d / 2) * (d / 2));

    // Vector perpendicular normalizado
    float nx = -dy / d;
    float ny = dx / d;

    // Determinar centro en una de las dos direcciones posibles
    float cx, cy;

    if (clockwise) {
        cx = mx + nx * h;
        cy = my + ny * h;
    } else {
        cx = mx - nx * h;
        cy = my - ny * h;
    }

    // Ángulos
    float start_angle = atan2(y0 - cy, x0 - cx);
    float end_angle = atan2(y1 - cy, x1 - cx);
    float total_angle = end_angle - start_angle;

    if (clockwise && total_angle > 0) {
        total_angle -= 2 * PI;
    } else if (!clockwise && total_angle < 0) {
        total_angle += 2 * PI;
    }

    for (int i = 1; i <= SEGMENTS; i++) {
        float angle = start_angle + total_angle * ((float)i / SEGMENTS);
        float x = cx + r * cos(angle);
        float y = cy + r * sin(angle);
        moveAxesWithFeedRate(x / STEPS_PER_MM_X, y / STEPS_PER_MM_Y, currentZ / STEPS_PER_MM_Z, rapidRate, true);
    }
}


void processGcode(const char* command) {
    // Comandos especiales para control de programa
    if (strncmp(command, "PROGRAM_START", 13) == 0) {
        startProgramStorage();
        return;
    }
    else if (strncmp(command, "PROGRAM_STOP", 12) == 0) {
        stopProgramStorage();
        return;
    }
    else if (strncmp(command, "PROGRAM_RUN", 11) == 0) {
        runProgram();
        return;
    }
    else if (strncmp(command, "PROGRAM_CLEAR", 13) == 0) {
        clearProgram();
        return;
    }
    else if (strncmp(command, "PROGRAM_PAUSE", 13) == 0) {
        pauseProgram();
        return;
    }
    else if (strncmp(command, "PROGRAM_NEXT", 12) == 0) {
        runNextLine();
        return;
    }
    else if (strncmp(command, "HELP", 4) == 0 || strncmp(command, "help", 4) == 0) {
        showHelp();
        return;
    }
    else if (strncmp(command, "QUEUE_STATUS", 12) == 0) {
        showQueueStatus();
        return;
    }
    else if (strncmp(command, "FIN", 3) == 0 || strncmp(command, "fin", 3) == 0) {
        if (isStoringProgram) {
            stopProgramStorage();
            return;
        }
    }
    
    // Si estamos en modo almacenamiento, agregar la línea al programa
    if (isStoringProgram) {
        if (addLineToProgram(command)) {
            sendUSBText("ok\r\n");
        } else {
            sendUSBText("error: buffer lleno\r\n");
        }
        return;
    }
    
    // Procesamiento normal de G-code
    char line_copy[100];
    strncpy(line_copy, command, sizeof(line_copy) - 1);
    line_copy[sizeof(line_copy) - 1] = '\0';
    
    uint8_t status = gc_execute_line(line_copy);
    
    // Enviar respuesta según el estándar GRBL
    if (status == STATUS_OK) {
        // Verificar si es un comando que requiere respuesta especial
        if (strncmp(command, "M114", 4) == 0) {
            // M114 - Reportar posición actual
            float xPos = currentX / (float)STEPS_PER_MM_X;
            float yPos = currentY / (float)STEPS_PER_MM_Y;
            float zPos = currentZ / (float)STEPS_PER_MM_Z;
            
            // Convertir a enteros para evitar problemas con printf float
            int x_int = (int)xPos;
            int x_dec = (int)((xPos - x_int) * 100);
            int y_int = (int)yPos;
            int y_dec = (int)((yPos - y_int) * 100);
            int z_int = (int)zPos;
            int z_dec = (int)((zPos - z_int) * 100);

            snprintf(outputBuffer, sizeof(outputBuffer), "X:%d.%02d Y:%d.%02d Z:%d.%02d\r\n", 
                   x_int, abs(x_dec), y_int, abs(y_dec), z_int, abs(z_dec));
            sendUSBText(outputBuffer);

        } else if (strncmp(command, "M503", 4) == 0) {
            // M503 - Mostrar configuración
            showConfiguration();
        }
    }
    
    // Enviar respuesta final usando el parser modular
    report_status_message(status);
}

void loop(void) {
    #if DEBUG_MESSAGES
    // Heartbeat para confirmar que el sistema está funcionando
    static uint32_t lastHeartbeat = 0;
    // Debug: mostrar tiempo actual
    char debugMsg[50];
    sprintf(debugMsg, "[DEBUG] Tiempo actual: %lu\r\n", currentTime);
    sendUSBText(debugMsg);
    

    // Heartbeat cada 5 segundos para confirmar que está vivo
    if (currentTime - lastHeartbeat > 5000) {
        lastHeartbeat = currentTime;
        sendUSBText("[HEARTBEAT] Sistema activo\r\n");
    }
    #endif
    
    // Procesar comandos USB CDC - SOLO cuando hay un comando completo
    if (usbCommandComplete) {
        #if DEBUG_MESSAGES
        // Debug: mostrar estado de variables
        char debugStatus[200];  // Buffer más grande para evitar overflow
        sprintf(debugStatus, "[DEBUG] usbCommandComplete=true, bufferIndex=%d, buffer=[%s]\r\n", 
                usbBufferIndex, usbBuffer);
        sendUSBText(debugStatus);
        #endif
        
        // Verificar que el buffer no esté vacío y contenga algo más que espacios
        bool hasValidCommand = false;
        for (int i = 0; i < usbBufferIndex; i++) {
            if (usbBuffer[i] != ' ' && usbBuffer[i] != '\t' && usbBuffer[i] != '\r' && usbBuffer[i] != '\n') {
                hasValidCommand = true;
                break;
            }
        }
        
        if (hasValidCommand && usbBufferIndex > 0) {
            #if DEBUG_MESSAGES
            // Debug: confirmar que llegó el comando (buffer más grande para evitar warning)
            sprintf(outputBuffer, ">>> [%s]\r\n", usbBuffer);
            sendUSBText(outputBuffer);
            memset(outputBuffer, 0, sizeof(outputBuffer));
            #endif
            
            processGcode(usbBuffer);
        }
        
        // IMPORTANTE: Resetear todo después de procesar (o intentar procesar)
        usbBufferIndex = 0;
        memset(usbBuffer, 0, sizeof(usbBuffer));
        usbCommandComplete = false;  // Asegurar que se resetee correctamente
    }
}

void setup(void) {
    // Activar DWT para microsegundos
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    
    // Habilitar drivers de motores (EN pins en LOW)
    enableSteppers();

    // Asegurar que LEDs estén apagados al inicio
    HAL_GPIO_WritePin(GPIOB, LED_HORARIO, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, LED_ANTIHORARIO, GPIO_PIN_RESET);
    
    // Inicializar parser G-code modular con callbacks
    gc_init();
}

// Nota: El callback USB CDC está implementado en usbd_cdc_if.c

// =============================================================================
// FUNCIONES DE CALLBACK PARA EL PARSER MODULAR
// =============================================================================

// Callback para realizar homing
void performHomingCallback(void) {
    performHoming();
}

// Callback para establecer posición
void setPositionCallback(float x, float y, float z, bool x_defined, bool y_defined, bool z_defined) {
    if (x_defined) {
        currentX = x * STEPS_PER_MM_X;
    }
    if (y_defined) {
        currentY = y * STEPS_PER_MM_Y;
    }
    if (z_defined) {
        currentZ = z * STEPS_PER_MM_Z;
    }
    char setMsg[100];
    
    // Convertir posiciones a enteros para evitar problemas con printf float
    float pos_x = currentX/(float)STEPS_PER_MM_X;
    float pos_y = currentY/(float)STEPS_PER_MM_Y;
    float pos_z = currentZ/(float)STEPS_PER_MM_Z;
    
    int x_int = (int)pos_x;
    int x_dec = (int)((pos_x - x_int) * 100);
    int y_int = (int)pos_y;
    int y_dec = (int)((pos_y - y_int) * 100);
    int z_int = (int)pos_z;
    int z_dec = (int)((pos_z - z_int) * 100);
    
    sprintf(outputBuffer, "Posición establecida: X%d.%02d Y%d.%02d Z%d.%02d\r\n",
           x_int, abs(x_dec), y_int, abs(y_dec), z_int, abs(z_dec));
    sendUSBText(outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));
}

// Callback para movimiento de ejes
void moveAxesCallback(float x, float y, float z, bool x_defined, bool y_defined, bool z_defined) {
    float target_x = x_defined ? x : NAN;
    float target_y = y_defined ? y : NAN;
    float target_z = z_defined ? z : NAN;
    
    // Usar función con feed rate por defecto (para compatibilidad)
    moveAxesWithFeedRate(target_x, target_y, target_z, currentFeedRate, false);
}

// Callback específico para movimiento rápido G0
void moveAxesRapidCallback(float x, float y, float z, bool x_defined, bool y_defined, bool z_defined) {
    float target_x = x_defined ? x : NAN;
    float target_y = y_defined ? y : NAN;
    float target_z = z_defined ? z : NAN;
    
    moveAxesWithFeedRate(target_x, target_y, target_z, rapidRate, true);
}

void moveAxesArcCallback(float x, float y, float r, int clockwise) {
    // Llamar a la función de movimiento de arco
    arc_move_r(x, y, r, clockwise);
}

// Callback específico para movimiento lineal G1 con feed rate
void moveAxesLinearCallback(float x, float y, float z, float feedRate, bool x_defined, bool y_defined, bool z_defined, bool f_defined) {
    float target_x = x_defined ? x : NAN;
    float target_y = y_defined ? y : NAN;
    float target_z = z_defined ? z : NAN;
    
    // Actualizar feed rate actual si se especifica
    if (f_defined && feedRate > 0) {
        currentFeedRate = feedRate;
    }
    
    moveAxesWithFeedRate(target_x, target_y, target_z, currentFeedRate, false);
}
// La función CDC_Receive_FS maneja la recepción de datos

// Función de compatibilidad UART (no se usa en este proyecto)
// El proyecto usa USB CDC para comunicación

// Función auxiliar para movimiento genérico (no utilizada actualmente)
// Se mantiene para compatibilidad futura

// Función para mostrar la configuración actual del sistema
void showConfiguration(void) {
    
    sendUSBText("=== CONFIGURACIÓN CNC ===\r\n");

    sprintf(outputBuffer, "Steps per mm X: %d\r\n", STEPS_PER_MM_X);
    sendUSBText((uint8_t*)outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));

    sprintf(outputBuffer, "Steps per mm Y: %d\r\n", STEPS_PER_MM_Y);
    sendUSBText((uint8_t*)outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));

    sprintf(outputBuffer, "Steps per mm Z: %d\r\n", STEPS_PER_MM_Z);
    sendUSBText((uint8_t*)outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));

    sprintf(outputBuffer, "Step delay: %d us\r\n", STEP_DELAY_US);
    sendUSBText((uint8_t*)outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));
    
    // Convertir feed rate actual a enteros
    int feed_int = (int)currentFeedRate;
    int feed_dec = (int)((currentFeedRate - feed_int) * 10);
    sprintf(outputBuffer, "Feed rate actual: %d.%d mm/min\r\n", feed_int, feed_dec);
    sendUSBText((uint8_t*)outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));

    // Convertir velocidades a enteros para mostrar
    int rapid_int = (int)rapidRate;
    int rapid_dec = (int)((rapidRate - rapid_int) * 10);
    sprintf(outputBuffer, "Velocidad rápida (G0): %d.%d mm/min\r\n", rapid_int, rapid_dec);
    sendUSBText((uint8_t*)outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));
    
    int max_int = (int)maxFeedRate;
    int max_dec = (int)((maxFeedRate - max_int) * 10);
    sprintf(outputBuffer, "Velocidad máxima: %d.%d mm/min\r\n", max_int, max_dec);
    sendUSBText((uint8_t*)outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));

    // Mostrar posición actual
    float xPos = currentX / (float)STEPS_PER_MM_X;
    float yPos = currentY / (float)STEPS_PER_MM_Y;
    float zPos = currentZ / (float)STEPS_PER_MM_Z;
    
    // Convertir a enteros para evitar problemas con printf float
    int x_int = (int)xPos;
    int x_dec = (int)((xPos - x_int) * 100);
    int y_int = (int)yPos;
    int y_dec = (int)((yPos - y_int) * 100);
    int z_int = (int)zPos;
    int z_dec = (int)((zPos - z_int) * 100);

    sprintf(outputBuffer, "Posición actual: X%d.%02d Y%d.%02d Z%d.%02d mm\r\n",
           x_int, abs(x_dec), y_int, abs(y_dec), z_int, abs(z_dec));
    sendUSBText((uint8_t*)outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));
    sendUSBText("=== FIN CONFIGURACIÓN ===\r\n");
}

// Función para verificar si un final de carrera está presionado
bool isEndstopPressed(char axis) {
    switch(axis) {
        case 'X':
            return (HAL_GPIO_ReadPin(GPIOB, X_MIN_PIN) == GPIO_PIN_RESET);
        case 'Y':
            return (HAL_GPIO_ReadPin(GPIOB, Y_MIN_PIN) == GPIO_PIN_RESET);
        case 'Z':
            return (HAL_GPIO_ReadPin(GPIOB, Z_MIN_PIN) == GPIO_PIN_RESET);
        default:
            return false;
    }
}

// Función de homing para todos los ejes
void performHoming(void) {
    char msg[80];
    
    // Enviar mensaje de inicio de homing
    sendUSBText("Iniciando secuencia de homingg...\r\n");
    
    // FASE 1: Movimiento rápido hacia los finales de carrera
    sprintf(outputBuffer, "Fase 1: Buscando finales de carrera...\r\n");
    // sendUSBText((uint8_t*)msg);
    sendUSBText(outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));

    // Homing del eje X
    sprintf(outputBuffer, "Homing eje X...\r\n");
    sendUSBText(outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));

    HAL_GPIO_WritePin(GPIOB, X_DIR_PIN, GPIO_PIN_RESET); // Dirección negativa
    // Mover hacia el final de carrera X (dirección negativa)
    while (!isEndstopPressed('X')) {
        X_stepOnce();
        delay_us(STEP_DELAY_US);
        //delay_us(STEP_DELAY_US / 2); // Movimiento más rápido para búsqueda inicial
        // HAL_GPIO_WritePin(GPIOB, X_DIR_PIN, GPIO_PIN_RESET); // Dirección negativa
    }
    
    // Retroceder un poco del final de carrera X
    HAL_GPIO_WritePin(GPIOB, X_DIR_PIN, GPIO_PIN_SET); // Dirección positiva
    for (int i = 0; i < 2*STEPS_PER_MM_X; i++) { // Retroceder 50 pasos
        //if (!isEndstopPressed('X')); // Salir cuando se libere el endstop
        X_stepOnce();
        delay_us(STEP_DELAY_US);
    }
    
    // FASE 2: Movimiento lento de precisión para X
    HAL_GPIO_WritePin(GPIOB, X_DIR_PIN, GPIO_PIN_RESET); // Dirección negativa nuevamente
    for (int i = 0; i < 2*STEPS_PER_MM_X; i++) { // Retroceder 50 pasos
        //if (!isEndstopPressed('X')); // Salir cuando se libere el endstop
        X_stepOnce();
        delay_us(STEP_DELAY_US * 3);
    }
    if (!isEndstopPressed('X')) {
        sprintf(outputBuffer, "Error: Final de carrera X no presionado\r\n");
        sendUSBText(outputBuffer);
        memset(outputBuffer, 0, sizeof(outputBuffer));
        // Activar interrupción o LED de error
        return; // Salir si no se presionó el endstop
    }
    
    currentX = 0; // Establecer posición home
    sprintf(outputBuffer, "Eje X en posición home\r\n");
    sendUSBText(outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));

    // Homing del eje Y
    sprintf(outputBuffer, "Homing eje Y...\r\n");
    sendUSBText(outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));

    // Mover hacia el final de carrera Y (dirección negativa)
    HAL_GPIO_WritePin(GPIOB, Y_DIR_PIN, GPIO_PIN_RESET); // Dirección negativa
    while (!isEndstopPressed('Y')) {
        Y_stepOnce();
        delay_us(STEP_DELAY_US);
        //delay_us(STEP_DELAY_US / 2); // Movimiento más rápido para búsqueda inicial
    }
    
    // Retroceder un poco del final de carrera Y
    HAL_GPIO_WritePin(GPIOB, Y_DIR_PIN, GPIO_PIN_SET); // Dirección positiva
    for (int i = 0; i < 2*STEPS_PER_MM_Y; i++) { // Retroceder 50 pasos
        //if (!isEndstopPressed('Y'));
        Y_stepOnce();
        delay_us(STEP_DELAY_US);
    }
    
    // FASE 2: Movimiento lento de precisión para Y
    HAL_GPIO_WritePin(GPIOB, Y_DIR_PIN, GPIO_PIN_RESET); // Dirección negativa nuevamente
    for (int i = 0; i < 2*STEPS_PER_MM_Y; i++) { // Retroceder 50 pasos
        //if (!isEndstopPressed('Y'));
        Y_stepOnce();
        delay_us(STEP_DELAY_US * 3); // Movimiento lento para precisión
    }
    if (!isEndstopPressed('Y')) {
        sprintf(outputBuffer, "Error: Final de carrera Y no presionado\r\n");
        sendUSBText(outputBuffer);
        memset(outputBuffer, 0, sizeof(outputBuffer));
        // Activar interrupción o LED de error
        return; // Salir si no se presionó el endstop
    }
    
    currentY = 0; // Establecer posición home
    sprintf(outputBuffer, "Eje Y en posición home\r\n");
    sendUSBText(outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));

    // Homing del eje Z
    sprintf(outputBuffer, "Homing eje Z...\r\n");
    sendUSBText(outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));

    // Mover hacia el final de carrera Z (dirección negativa)
    HAL_GPIO_WritePin(GPIOA, Z_DIR_PIN, GPIO_PIN_SET); // Dirección negativa
    while (!isEndstopPressed('Z')) {
        Z_stepOnce();
        delay_us(STEP_DELAY_US/2);
        //delay_us(STEP_DELAY_US / 2); // Movimiento más rápido para búsqueda inicial
    }
    
    // Retroceder un poco del final de carrera Z
    HAL_GPIO_WritePin(GPIOA, Z_DIR_PIN, GPIO_PIN_RESET); // Dirección positiva
    for (int i = 0; i < 2*STEPS_PER_MM_Z; i++) { // Retroceder 50 pasos
        // if (!isEndstopPressed('Z'));
        Z_stepOnce();
        delay_us(STEP_DELAY_US/2);
    }
    
    // FASE 2: Movimiento lento de precisión para Z
    HAL_GPIO_WritePin(GPIOA, Z_DIR_PIN, GPIO_PIN_SET); // Dirección negativa nuevamente
    for (int i = 0; i < 2*STEPS_PER_MM_Z; i++) { // Retroceder 50 pasos
        // if (!isEndstopPressed('Z'));
        Z_stepOnce();
        delay_us(STEP_DELAY_US * 3); // Movimiento lento para precisión
    }
    if (!isEndstopPressed('Z')) {
        sprintf(outputBuffer, "Error: Final de carrera Z no presionado\r\n");
        sendUSBText(outputBuffer);
        memset(outputBuffer, 0, sizeof(outputBuffer));
        // Activar interrupción o LED de error
        return; // Salir si no se presionó el endstop
    }
    
    currentZ = 0; // Establecer posición home
    sprintf(outputBuffer, "Eje Z en posición home\r\n");
    sendUSBText(outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));

    // Mensaje final
    sprintf(outputBuffer, "Homing completado. Todos los ejes en posición home.\r\n");
    sendUSBText(outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));
}

/**
  * @brief  Habilita todos los motores paso a paso
  * @retval None
  */
void enableSteppers(void) {
    // Habilitar drivers (EN LOW = habilitado para la mayoría de drivers A4988/DRV8825)
    HAL_GPIO_WritePin(GPIOB, X_EN_PIN, GPIO_PIN_RESET);  // Enable motor X
    HAL_GPIO_WritePin(GPIOB, Y_EN_PIN, GPIO_PIN_RESET);  // Enable motor Y
    HAL_GPIO_WritePin(GPIOA, Z_EN_PIN, GPIO_PIN_RESET);  // Enable motor Z
}

/**
  * @brief  Deshabilita todos los motores paso a paso
  * @retval None
  */
void disableSteppers(void) {
    // Deshabilitar drivers (EN HIGH = deshabilitado para la mayoría de drivers A4988/DRV8825)
    HAL_GPIO_WritePin(GPIOB, X_EN_PIN, GPIO_PIN_SET);    // Disable motor X
    HAL_GPIO_WritePin(GPIOB, Y_EN_PIN, GPIO_PIN_SET);    // Disable motor Y
    HAL_GPIO_WritePin(GPIOA, Z_EN_PIN, GPIO_PIN_SET);    // Disable motor Z
}

// =============================================================================
// FUNCIONES PARA MANEJO DE PROGRAMA G-CODE
// =============================================================================

/**
  * @brief  Inicia el modo de almacenamiento de programa
  * @retval None
  */
void startProgramStorage(void) {
    isStoringProgram = true;
    programLineCount = 0;
    isProgramLoaded = false;
    
    // Limpiar buffer de programa
    for (int i = 0; i < MAX_GCODE_LINES; i++) {
        memset(gcodeProgram[i], 0, MAX_LINE_LENGTH);
    }
    
    sendUSBText("Modo almacenamiento activado. Envie comandos G-code.\r\n");
    sendUSBText("Termine con 'FIN' o 'PROGRAM_STOP'\r\n");
    sendUSBText("ok\r\n");
}

/**
  * @brief  Detiene el modo de almacenamiento de programa
  * @retval None
  */
void stopProgramStorage(void) {
    isStoringProgram = false;
    
    if (programLineCount > 0) {
        isProgramLoaded = true;
        sprintf(outputBuffer, "Programa cargado: %d lineas almacenadas\r\n", programLineCount);
        sendUSBText(outputBuffer);
        memset(outputBuffer, 0, sizeof(outputBuffer));
        sendUSBText("Use 'PROGRAM_RUN' para ejecutar o 'PROGRAM_INFO' para ver detalles\r\n");
    } else {
        sendUSBText("No se almacenaron lineas\r\n");
    }
    
    sendUSBText("ok\r\n");
}

/**
  * @brief  Agrega una línea al programa almacenado
  * @param  line: Línea de G-code a agregar
  * @retval true si se agregó exitosamente, false si el buffer está lleno
  */
bool addLineToProgram(const char* line) {
    if (programLineCount >= MAX_GCODE_LINES) {
        return false; // Buffer lleno
    }
    
    // Copiar la línea, eliminando espacios al inicio y final
    const char* start = line;
    while (*start == ' ' || *start == '\t') start++; // Saltar espacios iniciales
    
    if (strlen(start) == 0) {
        return true; // Línea vacía, no la almacenamos pero no es error
    }
    
    strncpy(gcodeProgram[programLineCount], start, MAX_LINE_LENGTH - 1);
    gcodeProgram[programLineCount][MAX_LINE_LENGTH - 1] = '\0';
    
    // Eliminar \r\n del final si existen
    int len = strlen(gcodeProgram[programLineCount]);
    while (len > 0 && (gcodeProgram[programLineCount][len-1] == '\r' || 
                       gcodeProgram[programLineCount][len-1] == '\n')) {
        gcodeProgram[programLineCount][len-1] = '\0';
        len--;
    }
    
    programLineCount++;
    return true;
}

/**
  * @brief  Limpia el programa almacenado
  * @retval None
  */
void clearProgram(void) {
    programLineCount = 0;
    currentExecutingLine = 0;
    isProgramLoaded = false;
    isProgramRunning = false;
    isStoringProgram = false;
    
    // Limpiar buffer
    for (int i = 0; i < MAX_GCODE_LINES; i++) {
        memset(gcodeProgram[i], 0, MAX_LINE_LENGTH);
    }
    
    sendUSBText("Programa limpiado\r\n");
    sendUSBText("ok\r\n");
}


/**
  * @brief  Ejecuta el programa completo almacenado
  * @retval None
  */
void runProgram(void) {
    if (!isProgramLoaded || programLineCount == 0) {
        sprintf(outputBuffer, "error: No hay programa cargado\r\n");
        sendUSBText(outputBuffer);
        memset(outputBuffer, 0, sizeof(outputBuffer));
        return;
    }
    
    isProgramRunning = true;
    currentExecutingLine = 0;

    sprintf(outputBuffer, "Iniciando ejecucion del programa (%d lineas)\r\n", programLineCount);
    sendUSBText(outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));

    // Ejecutar todas las líneas secuencialmente
    for (currentExecutingLine = 0; currentExecutingLine < programLineCount; currentExecutingLine++) {
        if (!isProgramRunning) {
            sendUSBText((uint8_t*)"Programa detenido por el usuario\r\n");
            break;
        }

        sprintf(outputBuffer, "Ejecutando linea %d: %s\r\n", currentExecutingLine + 1, gcodeProgram[currentExecutingLine]);
        sendUSBText(outputBuffer);
        memset(outputBuffer, 0, sizeof(outputBuffer));

        // Crear una copia temporal para evitar recursión
        char temp_command[MAX_LINE_LENGTH];
        strncpy(temp_command, gcodeProgram[currentExecutingLine], MAX_LINE_LENGTH - 1);
        temp_command[MAX_LINE_LENGTH - 1] = '\0';
        
        // Ejecutar el comando directamente usando el parser modular
        uint8_t status = gc_execute_line(temp_command);
        
        // Mostrar resultado
        if (status == STATUS_OK) {
            sprintf(outputBuffer, "ok\r\n");
            sendUSBText(outputBuffer);
            memset(outputBuffer, 0, sizeof(outputBuffer));
        } else {
            sprintf(outputBuffer, "error: codigo %d en linea %d\r\n", status, currentExecutingLine + 1);
            sendUSBText(outputBuffer);
            memset(outputBuffer, 0, sizeof(outputBuffer));
            isProgramRunning = false;
            break;
        }
        
        // Pequeña pausa entre comandos para estabilidad
        HAL_Delay(10);
    }
    
    isProgramRunning = false;
    
    if (currentExecutingLine >= programLineCount) {
        sprintf(outputBuffer, "Programa completado exitosamente\r\n");
        sendUSBText(outputBuffer);
        memset(outputBuffer, 0, sizeof(outputBuffer));
    }

    sendUSBText("ok\r\n");
}

/**
  * @brief  Ejecuta la siguiente línea del programa
  * @retval None
  */
void runNextLine(void) {
    if (!isProgramLoaded || programLineCount == 0) {
        sprintf(outputBuffer, "error: No hay programa cargado\r\n");
        sendUSBText(outputBuffer);
        memset(outputBuffer, 0, sizeof(outputBuffer));
        return;
    }
    
    if (currentExecutingLine >= programLineCount) {
        sprintf(outputBuffer, "Programa completado\r\n");
        sendUSBText(outputBuffer);
        memset(outputBuffer, 0, sizeof(outputBuffer));
        isProgramRunning = false;
        return;
    }

    sprintf(outputBuffer, "Ejecutando linea %d: %s\r\n", currentExecutingLine + 1, gcodeProgram[currentExecutingLine]);
    sendUSBText(outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));
    // Ejecutar el comando
    char temp_command[MAX_LINE_LENGTH];
    strncpy(temp_command, gcodeProgram[currentExecutingLine], MAX_LINE_LENGTH - 1);
    temp_command[MAX_LINE_LENGTH - 1] = '\0';
    
    uint8_t status = gc_execute_line(temp_command);
    
    if (status == STATUS_OK) {
        sprintf(outputBuffer, "ok\r\n");
        sendUSBText(outputBuffer);
        memset(outputBuffer, 0, sizeof(outputBuffer));
        currentExecutingLine++;
    } else {
        sprintf(outputBuffer, "error: codigo %d en linea %d\r\n", status, currentExecutingLine + 1);
        sendUSBText(outputBuffer);
        memset(outputBuffer, 0, sizeof(outputBuffer));
        isProgramRunning = false;
    }
}

/**
  * @brief  Pausa la ejecución del programa
  * @retval None
  */
void pauseProgram(void) {
    isProgramRunning = false;
    sprintf(outputBuffer, "Programa pausado\r\n");
    sendUSBText(outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));
    sprintf(outputBuffer, "ok\r\n");
    sendUSBText(outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));
}

/**
  * @brief  Detiene completamente la ejecución del programa
  * @retval None
  */
void stopProgram(void) {
    isProgramRunning = false;
    currentExecutingLine = 0;
    sprintf(outputBuffer, "Programa detenido\r\n");
    sendUSBText(outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));
    sprintf(outputBuffer, "ok\r\n");
    sendUSBText(outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));
}

/**
  * @brief  Muestra la ayuda del sistema de programas G-code
  * @retval None
  */
void showHelp(void) {
    sendUSBText("\r\n=== AYUDA DEL SISTEMA CNC ===\r\n");
    sendUSBText("\r\nCOMANDOS DE PROGRAMA:\r\n");
    sendUSBText("PROGRAM_START  - Inicia modo almacenamiento de programa\r\n");
    sendUSBText("PROGRAM_STOP   - Detiene almacenamiento\r\n");
    sendUSBText("FIN            - Termina almacenamiento de programa\r\n");
    sendUSBText("PROGRAM_RUN    - Ejecuta programa completo\r\n");
    sendUSBText("PROGRAM_NEXT   - Ejecuta siguiente linea\r\n");
    sendUSBText("PROGRAM_PAUSE  - Pausa ejecucion\r\n");
    sendUSBText("PROGRAM_INFO   - Muestra informacion del programa\r\n");
    sendUSBText("PROGRAM_CLEAR  - Limpia programa almacenado\r\n");
    sendUSBText("QUEUE_STATUS   - Estado de cola de transmision USB\r\n");
    
    sendUSBText("\r\nCOMANDOS G-CODE BASICOS:\r\n");
    sendUSBText("G0 X Y Z       - Movimiento rapido\r\n");
    sendUSBText("G1 X Y Z F     - Movimiento lineal con feed rate\r\n");
    sendUSBText("G28            - Homing (ir a origen)\r\n");
    sendUSBText("G92 X Y Z      - Establecer posicion actual\r\n");
    sendUSBText("M17            - Habilitar motores\r\n");
    sendUSBText("M18 / M84      - Deshabilitar motores\r\n");
    sendUSBText("M114           - Reportar posicion actual\r\n");
    sendUSBText("M503           - Mostrar configuracion\r\n");
    
    sendUSBText("\r\nEJEMPLO DE USO:\r\n");
    sendUSBText("1. PROGRAM_START\r\n");
    sendUSBText("2. G28 (enviar)\r\n");
    sendUSBText("3. G0 X10 Y10 (enviar)\r\n");
    sendUSBText("4. G1 X20 Y20 F100 (enviar)\r\n");
    sendUSBText("5. FIN\r\n");
    sendUSBText("6. PROGRAM_RUN\r\n");
    
    sendUSBText("\r\n=== FIN AYUDA ===\r\n");
    sendUSBText("ok\r\n");
}

/**
  * @brief  Muestra el estado de la cola de transmisión USB CDC
  * @retval None
  */
void showQueueStatus(void) {

    sprintf(outputBuffer, "\r\n=== ESTADO COLA USB CDC ===\r\n");
    sendUSBText(outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));

    sprintf(outputBuffer, "Mensajes en cola: %d/%d\r\n", CDC_TxQueue_GetCount(), 10);
    sendUSBText(outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));

    sprintf(outputBuffer, "Cola llena: %s\r\n", CDC_TxQueue_IsFull() ? "SI" : "NO");
    sendUSBText(outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));

    sprintf(outputBuffer, "Método transmisión: ");
    sendUSBText(outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));
    
    switch (USB_TRANSMIT_METHOD) {
        case USB_METHOD_DIRECT:
            sendUSBText("DIRECTO\r\n");
            break;
        case USB_METHOD_RETRY:
            sendUSBText("REINTENTOS\r\n");
            break;
        case USB_METHOD_QUEUED:
            sendUSBText("COLA\r\n");
            break;
    }
    
    sendUSBText("===========================\r\n");
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
