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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
extern UART_HandleTypeDef huart1;
char inputBuffer[100];
int bufferIndex = 0;
bool commandComplete = false;
int32_t currentX = 0, currentY = 0, currentZ = 0;

// Buffer para recibir comandos por USB CDC
char usbBuffer[100];
int usbBufferIndex = 0;
bool usbCommandComplete = false;  // Cambiar a false para evitar procesamiento inicial

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
void moveAxes(float x, float y, float z);
float extractParam(const char* command, char param);
void processGcode(const char* command);
void readUSBCommands(void);
void X_move(int32_t steps, bool dir);
void Y_move(int32_t steps, bool dir);
void Z_move(int32_t steps, bool dir);
void X_stepOnce(void);
void Y_stepOnce(void);
void Z_stepOnce(void);
float extractParameter(const char* command, char param);
void performHoming(void);
bool isEndstopPressed(char axis);
void showConfiguration(void);
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

  // Inicialización similar al setup() de Arduino
  setup();

  // Envío inicial
  uint8_t mensaje[] = "G-code listo\r\n";
  CDC_Transmit_FS(mensaje, sizeof(mensaje) - 1);
  
  #if DEBUG_MESSAGES
  // Mensaje adicional de debug
  uint8_t debug_msg[] = "Sistema iniciado - Esperando comandos...\r\n";
  CDC_Transmit_FS(debug_msg, sizeof(debug_msg) - 1);
  #endif

  while (1)
  {
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

void readUSBCommands(void) {
    // Esta función se implementará con callback de USB CDC
    // Por ahora vacía, se procesará en el callback
}

float extractParameter(const char* command, char param) {
    char* ptr = strchr(command, param);
    if (ptr) {
        return atof(ptr + 1);
    }
    return NAN; // Not a Number
}

void moveAxes(float x, float y, float z) {
    // Convertir de milímetros a pasos usando valores específicos por eje
    int32_t xSteps = 0;
    int32_t ySteps = 0;
    int32_t zSteps = 0;
    bool xDir = true;
    bool yDir = true;
    bool zDir = true;
    
    if (!isnan(x)) {
        // Calcular pasos relativos para el eje X (mm → pasos)
        int32_t targetX = x * STEPS_PER_MM_X; // Convertir mm a pasos (79 steps/mm)
        xSteps = targetX - currentX;
        xDir = (xSteps >= 0);
        xSteps = abs(xSteps);
        currentX = targetX; // Actualizar posición actual
    }
    
    if (!isnan(y)) {
        // Calcular pasos relativos para el eje Y (mm → pasos)
        int32_t targetY = y * STEPS_PER_MM_Y; // Convertir mm a pasos (79 steps/mm)
        ySteps = targetY - currentY;
        yDir = (ySteps >= 0);
        ySteps = abs(ySteps);
        currentY = targetY; // Actualizar posición actual
    }
    
    if (!isnan(z)) {
        // Calcular pasos relativos para el eje Z (mm → pasos)
        int32_t targetZ = z * STEPS_PER_MM_Z; // Convertir mm a pasos (3930 steps/mm)
        zSteps = targetZ - currentZ;
        zDir = (zSteps >= 0);
        zSteps = abs(zSteps);
        currentZ = targetZ; // Actualizar posición actual
    }
    
    // Mover los motores
    if (xSteps > 0) {
        // Enviar información por USB CDC
        char msg[80];
        sprintf(msg, "Moviendo X: %.2fmm (%ld pasos), dir: %s\r\n", 
               !isnan(x) ? x : 0.0, xSteps, xDir ? "+" : "-");
        CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
        X_move(xSteps, xDir);
    }
    
    if (ySteps > 0) {
        // Enviar información por USB CDC
        char msg[80];
        sprintf(msg, "Moviendo Y: %.2fmm (%ld pasos), dir: %s\r\n", 
               !isnan(y) ? y : 0.0, ySteps, yDir ? "+" : "-");
        CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
        Y_move(ySteps, yDir);
    }
    
    if (zSteps > 0) {
        // Enviar información por USB CDC
        char msg[80];
        sprintf(msg, "Moviendo Z: %.2fmm (%ld pasos), dir: %s\r\n", 
               !isnan(z) ? z : 0.0, zSteps, zDir ? "+" : "-");
        CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
        Z_move(zSteps, zDir);
    }
}

void processGcode(const char* command) {
    // Enviar comando procesado
    // char msg[100];
    // sprintf(msg, "Procesando: %s\r\n", command);
    // CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
    
    // Procesar comando G0 o G1 (movimientos)
    if (strncmp(command, "G0", 2) == 0 || strncmp(command, "G1", 2) == 0) {
        // Procesar movimiento
        float xPos = extractParameter(command, 'X');
        float yPos = extractParameter(command, 'Y');
        float zPos = extractParameter(command, 'Z');
        
        // Mover los ejes
        moveAxes(xPos, yPos, zPos);
        
        CDC_Transmit_FS((uint8_t*)"OK\r\n", 4);
    }
    // Procesar comando G28 (homing)
    else if (strncmp(command, "G28", 3) == 0) {
        CDC_Transmit_FS((uint8_t*)"Ejecutando homing...\r\n", 22);
        performHoming();
        CDC_Transmit_FS((uint8_t*)"OK\r\n", 4);
    }
    // Comando M114 - Reportar posición actual
    else if (strncmp(command, "M114", 4) == 0) {
        char posMsg[100];
        float xPos = currentX / (float)STEPS_PER_MM_X;
        float yPos = currentY / (float)STEPS_PER_MM_Y;
        float zPos = currentZ / (float)STEPS_PER_MM_Z;
        sprintf(posMsg, "X:%.2f Y:%.2f Z:%.2f\r\n", xPos, yPos, zPos);
        CDC_Transmit_FS((uint8_t*)posMsg, strlen(posMsg));
        CDC_Transmit_FS((uint8_t*)"OK\r\n", 4);
    }
    // Comando M503 - Mostrar configuración
    else if (strncmp(command, "M503", 4) == 0) {
        showConfiguration();
        CDC_Transmit_FS((uint8_t*)"OK\r\n", 4);
    }
    else {
        CDC_Transmit_FS((uint8_t*)"Comando no reconocido\r\n", 23);
    }
}

void loop(void) {
    // Variables estáticas para controlar el spam de mensajes
    static uint32_t lastEndstopCheck = 0;
    static bool endstopXWasPressed = false; 
    static bool endstopYWasPressed = false;
    static bool endstopZWasPressed = false;
    
    uint32_t currentTime = HAL_GetTick();

    #if DEBUG_MESSAGES
    // Heartbeat para confirmar que el sistema está funcionando
    static uint32_t lastHeartbeat = 0;
    // Debug: mostrar tiempo actual
    char debugMsg[50];
    sprintf(debugMsg, "[DEBUG] Tiempo actual: %lu\r\n", currentTime);
    CDC_Transmit_FS((uint8_t*)debugMsg, strlen(debugMsg));
    

    // Heartbeat cada 5 segundos para confirmar que está vivo
    if (currentTime - lastHeartbeat > 5000) {
        lastHeartbeat = currentTime;
        CDC_Transmit_FS((uint8_t*)"[HEARTBEAT] Sistema activo\r\n", 29);
    }
    #endif
    // Verificar fines de carrera solo cada 10ms para evitar spam
    if (currentTime - lastEndstopCheck > 10) {
        lastEndstopCheck = currentTime;
        
        // Fin de carrera X - solo mensaje cuando cambia de estado
        bool endstopX = (HAL_GPIO_ReadPin(GPIOB, X_MIN_PIN) == GPIO_PIN_RESET);
        if (endstopX && !endstopXWasPressed) {
            CDC_Transmit_FS((uint8_t*)"Fin de carrera X activado\r\n", 27);
            // Opcional: detener motor X o hacer homing
            // X_move(100, false); // Retroceder 100 pasos
            endstopXWasPressed = true;
        } else if (!endstopX && endstopXWasPressed) {
            CDC_Transmit_FS((uint8_t*)"Fin de carrera X desactivado\r\n", 30);
            endstopXWasPressed = false;
        }
        
        // Fin de carrera Y
        bool endstopY = (HAL_GPIO_ReadPin(GPIOB, Y_MIN_PIN) == GPIO_PIN_RESET);
        if (endstopY && !endstopYWasPressed) {
            CDC_Transmit_FS((uint8_t*)"Fin de carrera Y activado\r\n", 27);
            endstopYWasPressed = true;
        } else if (!endstopY && endstopYWasPressed) {
            CDC_Transmit_FS((uint8_t*)"Fin de carrera Y desactivado\r\n", 30);
            endstopYWasPressed = false;
        }
        
        // Fin de carrera Z
        bool endstopZ = (HAL_GPIO_ReadPin(GPIOB, Z_MIN_PIN) == GPIO_PIN_RESET);
        if (endstopZ && !endstopZWasPressed) {
            CDC_Transmit_FS((uint8_t*)"Fin de carrera Z activado\r\n", 27);
            endstopZWasPressed = true;
        } else if (!endstopZ && endstopZWasPressed) {
            CDC_Transmit_FS((uint8_t*)"Fin de carrera Z desactivado\r\n", 30);
            endstopZWasPressed = false;
        }
    }

    // Procesar comandos USB CDC - SOLO cuando hay un comando completo
    if (usbCommandComplete) {
        #if DEBUG_MESSAGES
        // Debug: mostrar estado de variables
        char debugStatus[200];  // Buffer más grande para evitar overflow
        sprintf(debugStatus, "[DEBUG] usbCommandComplete=true, bufferIndex=%d, buffer=[%s]\r\n", 
                usbBufferIndex, usbBuffer);
        CDC_Transmit_FS((uint8_t*)debugStatus, strlen(debugStatus));
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
            char debugMsg[120];
            sprintf(debugMsg, ">>> [%s]\r\n", usbBuffer);
            CDC_Transmit_FS((uint8_t*)debugMsg, strlen(debugMsg));
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
    HAL_GPIO_WritePin(GPIOB, X_EN_PIN, GPIO_PIN_RESET);  // Habilita driver X
    HAL_GPIO_WritePin(GPIOB, Y_EN_PIN, GPIO_PIN_RESET);  // Habilita driver Y
    HAL_GPIO_WritePin(GPIOA, Z_EN_PIN, GPIO_PIN_RESET);  // Habilita driver Z

    // Asegurar que LEDs estén apagados al inicio
    HAL_GPIO_WritePin(GPIOB, LED_HORARIO, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, LED_ANTIHORARIO, GPIO_PIN_RESET);
}

// Nota: El callback USB CDC está implementado en usbd_cdc_if.c
// La función CDC_Receive_FS maneja la recepción de datos

// Función compatible con el código original (mantener para compatibilidad)
float extractParam(const char* command, char param) {
    return extractParameter(command, param);
}

// Función de compatibilidad UART (no se usa en este proyecto)
// El proyecto usa USB CDC para comunicación

// Función auxiliar para movimiento genérico (no utilizada actualmente)
// Se mantiene para compatibilidad futura

// Función para mostrar la configuración actual del sistema
void showConfiguration(void) {
    char msg[200];
    
    CDC_Transmit_FS((uint8_t*)"=== CONFIGURACIÓN CNC ===\r\n", 28);
    
    sprintf(msg, "Steps per mm X: %d\r\n", STEPS_PER_MM_X);
    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
    
    sprintf(msg, "Steps per mm Y: %d\r\n", STEPS_PER_MM_Y);
    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
    
    sprintf(msg, "Steps per mm Z: %d\r\n", STEPS_PER_MM_Z);
    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
    
    sprintf(msg, "Step delay: %d us\r\n", STEP_DELAY_US);
    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
    
    // Mostrar posición actual
    float xPos = currentX / (float)STEPS_PER_MM_X;
    float yPos = currentY / (float)STEPS_PER_MM_Y;
    float zPos = currentZ / (float)STEPS_PER_MM_Z;
    sprintf(msg, "Posición actual: X%.2f Y%.2f Z%.2f mm\r\n", xPos, yPos, zPos);
    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
    
    CDC_Transmit_FS((uint8_t*)"=== FIN CONFIGURACIÓN ===\r\n", 28);
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
    CDC_Transmit_FS((uint8_t*)"Iniciando secuencia de homingg...\r\n", 34);
    
    // FASE 1: Movimiento rápido hacia los finales de carrera
    sprintf(msg, "Fase 1: Buscando finales de carrera...\r\n");
    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
    
    // Homing del eje X
    sprintf(msg, "Homing eje X...\r\n");
    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));

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
    for (int i = 0; i < 10; i++) { // Retroceder 50 pasos
        if (!isEndstopPressed('X')) break; // Salir cuando se libere el endstop
        X_stepOnce();
        delay_us(STEP_DELAY_US);
    }
    
    // FASE 2: Movimiento lento de precisión para X
    HAL_GPIO_WritePin(GPIOB, X_DIR_PIN, GPIO_PIN_RESET); // Dirección negativa nuevamente
    while (!isEndstopPressed('X')) {
        X_stepOnce();
        delay_us(STEP_DELAY_US * 3); // Movimiento lento para precisión
    }
    
    currentX = 0; // Establecer posición home
    sprintf(msg, "Eje XX en posición home\r\n");
    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
    
    // Homing del eje Y
    sprintf(msg, "Homing eje Y...\r\n");
    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
    
    // Mover hacia el final de carrera Y (dirección negativa)
    HAL_GPIO_WritePin(GPIOB, Y_DIR_PIN, GPIO_PIN_RESET); // Dirección negativa
    while (!isEndstopPressed('Y')) {
        Y_stepOnce();
        delay_us(STEP_DELAY_US);
        //delay_us(STEP_DELAY_US / 2); // Movimiento más rápido para búsqueda inicial
    }
    
    // Retroceder un poco del final de carrera Y
    HAL_GPIO_WritePin(GPIOB, Y_DIR_PIN, GPIO_PIN_SET); // Dirección positiva
    for (int i = 0; i < 10; i++) { // Retroceder 50 pasos
        if (!isEndstopPressed('Y')) break;
        Y_stepOnce();
        delay_us(STEP_DELAY_US);
    }
    
    // FASE 2: Movimiento lento de precisión para Y
    HAL_GPIO_WritePin(GPIOB, Y_DIR_PIN, GPIO_PIN_RESET); // Dirección negativa nuevamente
    while (!isEndstopPressed('Y')) {
        Y_stepOnce();
        delay_us(STEP_DELAY_US * 3); // Movimiento lento para precisión
    }
    
    currentY = 0; // Establecer posición home
    sprintf(msg, "Eje Y en posición home\r\n");
    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
    
    // Homing del eje Z
    sprintf(msg, "Homing eje Z...\r\n");
    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
    
    // Mover hacia el final de carrera Z (dirección negativa)
    HAL_GPIO_WritePin(GPIOA, Z_DIR_PIN, GPIO_PIN_SET); // Dirección negativa
    while (!isEndstopPressed('Z')) {
        Z_stepOnce();
        delay_us(STEP_DELAY_US);
        //delay_us(STEP_DELAY_US / 2); // Movimiento más rápido para búsqueda inicial
    }
    
    // Retroceder un poco del final de carrera Z
    HAL_GPIO_WritePin(GPIOA, Z_DIR_PIN, GPIO_PIN_RESET); // Dirección positiva
    for (int i = 0; i < 10; i++) { // Retroceder 50 pasos
        if (!isEndstopPressed('Z')) break;
        Z_stepOnce();
        delay_us(STEP_DELAY_US);
    }
    
    // FASE 2: Movimiento lento de precisión para Z
    HAL_GPIO_WritePin(GPIOA, Z_DIR_PIN, GPIO_PIN_SET); // Dirección negativa nuevamente
    while (!isEndstopPressed('Z')) {
        Z_stepOnce();
        delay_us(STEP_DELAY_US * 3); // Movimiento lento para precisión
    }
    
    currentZ = 0; // Establecer posición home
    sprintf(msg, "Eje Z en posición home\r\n");
    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
    
    // Mensaje final
    CDC_Transmit_FS((uint8_t*)"Homing completado. Todos los ejes en posición home.\r\n", 54);
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
