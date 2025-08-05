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
#include "config.h"
#include "motion.h"
#include "planner.h"
//#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
extern UART_HandleTypeDef huart1;
// Declarar buffer global (accesible desde motion.c)
char outputBuffer[OUTPUT_BUFFER_SIZE];
int bufferIndex = 0;
bool commandComplete = false;
int32_t currentX, currentY, currentZ;

// Variables para control de velocidad y feed rate
float currentFeedRate = DEFAULT_FEED_RATE;     // Feed rate en mm/min (valor por defecto)
float rapidRate = DEFAULT_RAPID_RATE;          // Velocidad rápida para G0 en mm/min
float maxFeedRate = DEFAULT_MAX_FEED_RATE;     // Velocidad máxima permitida

// Buffer para recibir comandos por USB CDC
char usbBuffer[USB_BUFFER_SIZE];
int usbBufferIndex = 0;
bool usbCommandComplete = false;  // Cambiar a false para evitar procesamiento inicial

// Sistema de almacenamiento de programa G-code
char gcodeProgram[MAX_GCODE_LINES][MAX_LINE_LENGTH];  // Buffer para almacenar el programa
int programLineCount = 0;          // Número de líneas actualmente almacenadas
int currentExecutingLine = 0;      // Línea que se está ejecutando actualmente
bool isStoringProgram = false;     // Flag para indicar si estamos en modo almacenamiento
bool isProgramLoaded = false;      // Flag para indicar si hay un programa cargado
bool isProgramRunning = false;     // Flag para indicar si el programa se está ejecutando

const uint16_t STEP_DELAY_VALUE = STEP_DELAY_US;  // Usar valor de config.h
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void loop(void);
void setup(void);
// void moveAxes(float x, float y, float z);
void processGcode(const char* command);
void performHoming(void);
bool isEndstopPressed(char axis);
void showConfiguration(void);
void report_status_message(uint8_t status_code);
void endstop_error_handler(char axis);

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
void showProgramStatus(void); // Nueva función para estado del programa

// Funciones para integración con planner
void showPlannerStatus(void);
void processPlannerBuffer(void);
void processProgram(void);  // Nueva función para ejecutar programa progresivamente

// Variables de control del planner
bool plannerEnabled = true;  // Control para habilitar/deshabilitar planner
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

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

    // Inicializar cola de transmisión USB CDC
    CDC_TxQueue_Init();

    // Inicialización similar al setup() de Arduino
    setup();

    // Led de encendido inicial
    HAL_GPIO_WritePin(GPIOB, LED_CHECK, GPIO_PIN_SET);  // Encender
    // Envío inicial usando cola
    CDC_Transmit_Queued((uint8_t*)"G-code listo\r\n", 14); 

    #if DEBUG_MESSAGES
    // Mensaje adicional de debug
    CDC_Transmit_Queued((uint8_t*)"Sistema iniciado - Esperando comandos...\r\n", 42);
    #endif
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Procesar cola de transmisión USB CDC
    CDC_TxQueue_Process();
    
    // Equivalente al loop() de Arduino
    loop();
    
    // Pausa optimizada para reducir carga del procesador y terminal
    // HAL_Delay(50);  // 50ms = 20Hz, reduce carga significativamente
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
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
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */
//
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
//
  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
//
  /* USER CODE END USART2_Init 2 */

}

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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 PB3 PB4
                           PB6 PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
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
    else if (strncmp(command, "PROGRAM_STATUS", 14) == 0) {
        showProgramStatus();
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
    else if (strncmp(command, "PLANNER_STATUS", 14) == 0) {
        showPlannerStatus();
        return;
    }
    else if (strncmp(command, "PLANNER_ENABLE", 14) == 0) {
        plannerEnabled = true;
        sendUSBText("Planner habilitado\r\n");
        return;
    }
    else if (strncmp(command, "PLANNER_DISABLE", 15) == 0) {
        plannerEnabled = false;
        planner_synchronize(); // Esperar que termine el buffer actual
        sendUSBText("Planner deshabilitado\r\n");
        return;
    }
    else if (strncmp(command, "PLANNER_SYNC", 12) == 0) {
        planner_synchronize();
        sendUSBText("Planner sincronizado\r\n");
        return;
    }
    else if (strncmp(command, "PLANNER_RESET", 13) == 0) {
        planner_reset();
        sendUSBText("Planner reiniciado\r\n");
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
    uint32_t currentTime = HAL_GetTick();  // Declarar currentTime
    
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
    
    // Procesar buffer del planner (alta prioridad)
    if (plannerEnabled) {
        processPlannerBuffer();
    }
    
    // Procesar programa en ejecución (si corresponde)
    if (isProgramRunning) {
        processProgram();
    }
    
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
    HAL_GPIO_WritePin(GPIOB, LED_CHECK, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, LED_ERROR, GPIO_PIN_RESET);

    // Secuencia de 3 parpadeos de los LEDs de chequeo y error
    for (int i = 0; i < 3; i++) {
        HAL_GPIO_WritePin(GPIOB, LED_CHECK, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, LED_ERROR, GPIO_PIN_SET);
        HAL_Delay(150);
        HAL_GPIO_WritePin(GPIOB, LED_CHECK, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, LED_ERROR, GPIO_PIN_RESET);
        HAL_Delay(150);
    }
    
    // Inicializar parser G-code modular con callbacks
    gc_init();
    
    // Inicializar planner lookahead
    planner_init();
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
    
    // Actualizar posición del planner también
    if (plannerEnabled) {
        float planner_pos[3];
        planner_get_current_position(planner_pos);
        
        if (x_defined) planner_pos[0] = x;
        if (y_defined) planner_pos[1] = y;
        if (z_defined) planner_pos[2] = z;
        
        planner_set_current_position(planner_pos);
    }
    
    // char setMsg[100];
    
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

// Callback para movimiento de ejes - Integrado con planner
void moveAxesCallback(float x, float y, float z, bool x_defined, bool y_defined, bool z_defined) {
    if (plannerEnabled) {
        // Usar el planner para movimientos suavizados
        float target[3];
        planner_get_current_position(target);
        
        if (x_defined) target[0] = x;
        if (y_defined) target[1] = y;
        if (z_defined) target[2] = z;
        
        // Agregar al buffer del planner como movimiento lineal
        if (!planner_buffer_line(target, currentFeedRate, false)) {
            sendUSBText("error: buffer planner lleno\r\n");
        }
    } else {
        // Usar movimiento directo (modo compatibilidad)
        float target_x = x_defined ? x : NAN;
        float target_y = y_defined ? y : NAN;
        float target_z = z_defined ? z : NAN;
        
        moveAxesWithFeedRate(target_x, target_y, target_z, currentFeedRate, false);
    }
}

// Callback específico para movimiento rápido G0 - Integrado con planner
void moveAxesRapidCallback(float x, float y, float z, bool x_defined, bool y_defined, bool z_defined) {
    if (plannerEnabled) {
        // Usar el planner para movimientos suavizados
        float target[3];
        planner_get_current_position(target);
        
        if (x_defined) target[0] = x;
        if (y_defined) target[1] = y;
        if (z_defined) target[2] = z;
        
        // Agregar al buffer del planner como movimiento rápido
        if (!planner_buffer_line(target, rapidRate, true)) {
            sendUSBText("error: buffer planner lleno\r\n");
        }
    } else {
        // Usar movimiento directo (modo compatibilidad)
        float target_x = x_defined ? x : NAN;
        float target_y = y_defined ? y : NAN;
        float target_z = z_defined ? z : NAN;
        
        moveAxesWithFeedRate(target_x, target_y, target_z, rapidRate, true);
    }
}

void moveAxesArcCallback(float x, float y, float r, bool clockwise) {
    if (plannerEnabled) {
        // Usar el planner para arcos suavizados
        float target[3];
        planner_get_current_position(target);
        target[0] = x;
        target[1] = y;
        // Z se mantiene igual para arcos 2D
        
        float offset[2] = {r, 0}; // En modo R, solo se usa el primer elemento
        
        // Agregar arco al buffer del planner
        if (!planner_buffer_arc(target, offset, clockwise, true, currentFeedRate)) {
            sendUSBText("error: buffer planner lleno o arco inválido\r\n");
        }
    } else {
        // Usar movimiento directo (modo compatibilidad)
        arc_move_r(x, y, r, clockwise);
    }
}

// Callback específico para movimiento lineal G1 con feed rate - Integrado con planner
void moveAxesLinearCallback(float x, float y, float z, float feedRate, bool x_defined, bool y_defined, bool z_defined, bool f_defined) {
    // Actualizar feed rate actual si se especifica
    if (f_defined && feedRate > 0) {
        currentFeedRate = feedRate;
    }
    
    if (plannerEnabled) {
        // Usar el planner para movimientos suavizados
        float target[3];
        planner_get_current_position(target);
        
        if (x_defined) target[0] = x;
        if (y_defined) target[1] = y;
        if (z_defined) target[2] = z;
        
        // Agregar al buffer del planner como movimiento lineal
        if (!planner_buffer_line(target, currentFeedRate, false)) {
            sendUSBText("error: buffer planner lleno\r\n");
        }
    } else {
        // Usar movimiento directo (modo compatibilidad)
        float target_x = x_defined ? x : NAN;
        float target_y = y_defined ? y : NAN;
        float target_z = z_defined ? z : NAN;
        
        moveAxesWithFeedRate(target_x, target_y, target_z, currentFeedRate, false);
    }
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
    sendUSBText(outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));

    sprintf(outputBuffer, "Steps per mm Y: %d\r\n", STEPS_PER_MM_Y);
    sendUSBText(outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));

    sprintf(outputBuffer, "Steps per mm Z: %d\r\n", STEPS_PER_MM_Z);
    sendUSBText(outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));

    sprintf(outputBuffer, "Step delay: %d us\r\n", STEP_DELAY_US);
    sendUSBText(outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));
    
    // Convertir feed rate actual a enteros
    int feed_int = (int)currentFeedRate;
    int feed_dec = (int)((currentFeedRate - feed_int) * 10);
    sprintf(outputBuffer, "Feed rate actual: %d.%d mm/min\r\n", feed_int, feed_dec);
    sendUSBText(outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));

    // Convertir velocidades a enteros para mostrar
    int rapid_int = (int)rapidRate;
    int rapid_dec = (int)((rapidRate - rapid_int) * 10);
    sprintf(outputBuffer, "Velocidad rápida (G0): %d.%d mm/min\r\n", rapid_int, rapid_dec);
    sendUSBText(outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));
    
    int max_int = (int)maxFeedRate;
    int max_dec = (int)((maxFeedRate - max_int) * 10);
    sprintf(outputBuffer, "Velocidad máxima: %d.%d mm/min\r\n", max_int, max_dec);
    sendUSBText(outputBuffer);
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
    sendUSBText(outputBuffer);
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
    // char msg[80];
    // HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
    
    // Sincronizar planner antes de hacer homing
    if (plannerEnabled) {
        planner_synchronize();
    }
    
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
    CDC_TxQueue_Process();

    // Mover hacia el final de carrera X (dirección negativa)
    HAL_GPIO_WritePin(GPIOB, X_DIR_PIN, GPIO_PIN_RESET); // Dirección negativa
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
        sendUSBText(outputBuffer);CDC_TxQueue_Process();
        memset(outputBuffer, 0, sizeof(outputBuffer));
        // Activar interrupción o LED de error
        return; // Salir si no se presionó el endstop
    }
    
    currentX = 0; // Establecer posición home
    sprintf(outputBuffer, "Eje X en posición home\r\n");
    sendUSBText(outputBuffer);CDC_TxQueue_Process();
    memset(outputBuffer, 0, sizeof(outputBuffer));

    // Homing del eje Y
    sprintf(outputBuffer, "Homing eje Y...\r\n");
    sendUSBText(outputBuffer);CDC_TxQueue_Process();
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
        sendUSBText(outputBuffer);CDC_TxQueue_Process();
        memset(outputBuffer, 0, sizeof(outputBuffer));
        // Activar interrupción o LED de error
        return; // Salir si no se presionó el endstop
    }
    
    currentY = 0; // Establecer posición home
    sprintf(outputBuffer, "Eje Y en posición home\r\n");
    sendUSBText(outputBuffer);CDC_TxQueue_Process();
    memset(outputBuffer, 0, sizeof(outputBuffer));

    // Homing del eje Z
    sprintf(outputBuffer, "Homing eje Z...\r\n");
    sendUSBText(outputBuffer);CDC_TxQueue_Process();
    memset(outputBuffer, 0, sizeof(outputBuffer));

    // Mover hacia el final de carrera Z (dirección negativa)
    HAL_GPIO_WritePin(GPIOA, Z_DIR_PIN, GPIO_PIN_SET); // Dirección negativa
    while (!isEndstopPressed('Z')) {
        Z_stepOnce();
        delay_us(STEP_DELAY_US/3);
        //delay_us(STEP_DELAY_US / 2); // Movimiento más rápido para búsqueda inicial
    }
    
    // Retroceder un poco del final de carrera Z
    HAL_GPIO_WritePin(GPIOA, Z_DIR_PIN, GPIO_PIN_RESET); // Dirección positiva
    for (int i = 0; i < 1*STEPS_PER_MM_Z; i++) { // Retroceder 50 pasos
        // if (!isEndstopPressed('Z'));
        Z_stepOnce();
        delay_us(STEP_DELAY_US/3);
    }
    
    // FASE 2: Movimiento lento de precisión para Z
    HAL_GPIO_WritePin(GPIOA, Z_DIR_PIN, GPIO_PIN_SET); // Dirección negativa nuevamente
    for (int i = 0; i < 1*STEPS_PER_MM_Z; i++) { // Retroceder 50 pasos
        // if (!isEndstopPressed('Z'));
        Z_stepOnce();
        delay_us(STEP_DELAY_US/3); // Movimiento lento para precisión
    }
    if (!isEndstopPressed('Z')) {
        sprintf(outputBuffer, "Error: Final de carrera Z no presionado\r\n");
        sendUSBText(outputBuffer);CDC_TxQueue_Process();
        memset(outputBuffer, 0, sizeof(outputBuffer));
        // Activar interrupción o LED de error
        return; // Salir si no se presionó el endstop
    }
    
    currentZ = 0; // Establecer posición home
    sprintf(outputBuffer, "Eje Z en posición home\r\n");
    sendUSBText(outputBuffer);CDC_TxQueue_Process();
    memset(outputBuffer, 0, sizeof(outputBuffer));

    // Mensaje final
    sprintf(outputBuffer, "Homing completado. Todos los ejes en posición home.\r\n");
    sendUSBText(outputBuffer);CDC_TxQueue_Process();
    memset(outputBuffer, 0, sizeof(outputBuffer));

    // Actualizar posición del planner a home
    if (plannerEnabled) {
        float home_pos[3] = {0.0f, 0.0f, 0.0f};
        planner_set_current_position(home_pos);
    }

    // Rehabilitar interrupciones
    // HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
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
    //sendUSBText("ok\r\n");
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
    //sendUSBText("ok\r\n");
}


/**
  * @brief  Ejecuta el programa completo almacenado usando el planner
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
    
    sendUSBText("Programa en ejecucion. Use PROGRAM_PAUSE para pausar.\r\n");
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
        // sprintf(outputBuffer, "ok\r\n");
        // sendUSBText(outputBuffer);
        // memset(outputBuffer, 0, sizeof(outputBuffer));
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
    sendUSBText("PROGRAM_STATUS - Estado del programa actual\r\n");
    sendUSBText("PROGRAM_INFO   - Muestra informacion del programa\r\n");
    sendUSBText("PROGRAM_CLEAR  - Limpia programa almacenado\r\n");
    sendUSBText("QUEUE_STATUS   - Estado de cola de transmision USB\r\n");
    sendUSBText("PLANNER_STATUS - Estado del planner de movimientos\r\n");
    sendUSBText("PLANNER_ENABLE - Habilita el planner lookahead\r\n");
    sendUSBText("PLANNER_DISABLE- Deshabilita el planner lookahead\r\n");
    sendUSBText("PLANNER_SYNC   - Sincroniza el planner (espera buffer vacio)\r\n");
    sendUSBText("PLANNER_RESET  - Reinicia el planner\r\n");
    
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

/**
  * @brief  Muestra el estado del planner lookahead
  * @retval None
  */
void showPlannerStatus(void) {
    uint8_t buffer_usage;
    uint32_t blocks_processed;
    bool is_running;
    
    // Obtener estadísticas del planner
    planner_get_statistics(&buffer_usage, &blocks_processed, &is_running);
    
    sprintf(outputBuffer, "\r\n=== ESTADO PLANNER LOOKAHEAD ===\r\n");
    sendUSBText(outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));
    
    sprintf(outputBuffer, "Estado: %s\r\n", plannerEnabled ? "HABILITADO" : "DESHABILITADO");
    sendUSBText(outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));
    
    sprintf(outputBuffer, "Ejecutándose: %s\r\n", is_running ? "SI" : "NO");
    sendUSBText(outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));
    
    sprintf(outputBuffer, "Buffer: %d/%d bloques (%d%% uso)\r\n", 
           planner_get_buffer_count(), PLANNER_BUFFER_SIZE, buffer_usage);
    sendUSBText(outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));
    
    sprintf(outputBuffer, "Bloques procesados: %lu\r\n", blocks_processed);
    sendUSBText(outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));
    
    // Obtener posición actual del planner
    float planner_pos[3];
    planner_get_current_position(planner_pos);
    
    int x_int = (int)planner_pos[0];
    int x_dec = (int)((planner_pos[0] - x_int) * 100);
    int y_int = (int)planner_pos[1];
    int y_dec = (int)((planner_pos[1] - y_int) * 100);
    int z_int = (int)planner_pos[2];
    int z_dec = (int)((planner_pos[2] - z_int) * 100);
    
    sprintf(outputBuffer, "Posición planner: X%d.%02d Y%d.%02d Z%d.%02d mm\r\n",
           x_int, abs(x_dec), y_int, abs(y_dec), z_int, abs(z_dec));
    sendUSBText(outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));
    
    sprintf(outputBuffer, "Configuración:\r\n");
    sendUSBText(outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));
    
    sprintf(outputBuffer, "  - Buffer size: %d bloques\r\n", PLANNER_BUFFER_SIZE);
    sendUSBText(outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));
    
    sprintf(outputBuffer, "  - Junction deviation: %.2f mm\r\n", JUNCTION_DEVIATION);
    sendUSBText(outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));
    
    int acc_int = (int)ACCELERATION;
    int acc_dec = (int)((ACCELERATION - acc_int) * 10);
    sprintf(outputBuffer, "  - Aceleración: %d.%d mm/min²\r\n", acc_int, acc_dec);
    sendUSBText(outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));
    
    sendUSBText("================================\r\n");
}

/**
  * @brief  Muestra el estado del programa en ejecución
  * @retval None
  */
void showProgramStatus(void) {
    sprintf(outputBuffer, "\r\n=== ESTADO PROGRAMA G-CODE ===\r\n");
    sendUSBText(outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));
    
    sprintf(outputBuffer, "Programa cargado: %s\r\n", isProgramLoaded ? "SI" : "NO");
    sendUSBText(outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));
    
    sprintf(outputBuffer, "Programa ejecutándose: %s\r\n", isProgramRunning ? "SI" : "NO");
    sendUSBText(outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));
    
    if (isProgramLoaded) {
        sprintf(outputBuffer, "Total de líneas: %d\r\n", programLineCount);
        sendUSBText(outputBuffer);
        memset(outputBuffer, 0, sizeof(outputBuffer));
        
        sprintf(outputBuffer, "Línea actual: %d\r\n", currentExecutingLine + 1);
        sendUSBText(outputBuffer);
        memset(outputBuffer, 0, sizeof(outputBuffer));
        
        int progress = (programLineCount > 0) ? (currentExecutingLine * 100) / programLineCount : 0;
        sprintf(outputBuffer, "Progreso: %d%%\r\n", progress);
        sendUSBText(outputBuffer);
        memset(outputBuffer, 0, sizeof(outputBuffer));
        
        if (isProgramRunning && currentExecutingLine < programLineCount) {
            sprintf(outputBuffer, "Próxima línea: %s\r\n", gcodeProgram[currentExecutingLine]);
            sendUSBText(outputBuffer);
            memset(outputBuffer, 0, sizeof(outputBuffer));
        }
    }
    
    sendUSBText("=============================\r\n");
}

/**
  * @brief  Procesa el buffer del planner en el loop principal
  * @retval None
  */
void processPlannerBuffer(void) {
    // Procesar hasta 2 bloques por ciclo para mantener fluidez
    for (int i = 0; i < 2; i++) {
        if (!planner_process_next_block()) {
            break; // No hay más bloques para procesar
        }
    }
}

/**
  * @brief  Procesa la ejecución del programa de manera no bloqueante
  * @retval None
  */
void processProgram(void) {
    static uint32_t lastProgramTime = 0;
    uint32_t currentTime = HAL_GetTick();
    
    // Verificar si hay líneas pendientes y si el buffer tiene espacio
    if (currentExecutingLine >= programLineCount) {
        // Programa completado
        if (plannerEnabled) {
            // Esperar que el planner termine todos los bloques
            if (planner_get_buffer_count() == 0) {
                sprintf(outputBuffer, "Programa completado exitosamente\r\n");
                sendUSBText(outputBuffer);
                memset(outputBuffer, 0, sizeof(outputBuffer));
                isProgramRunning = false;
            }
        } else {
            sprintf(outputBuffer, "Programa completado exitosamente\r\n");
            sendUSBText(outputBuffer);
            memset(outputBuffer, 0, sizeof(outputBuffer));
            isProgramRunning = false;
        }
        return;
    }
    
    // Control de velocidad - no ejecutar comandos muy rápido
    // if (currentTime - lastProgramTime < 50) {  // Mínimo 50ms entre comandos
    //     return;
    // }
    
    // Verificar si el buffer del planner tiene espacio (si está habilitado)
    if (plannerEnabled && planner_get_buffer_count() >= (PLANNER_BUFFER_SIZE - 2)) {
        // Buffer casi lleno, esperar antes de agregar más comandos
        return;
    }
    
    // Ejecutar siguiente línea
    sprintf(outputBuffer, "Ejecutando linea %d: %s\r\n", currentExecutingLine + 1, gcodeProgram[currentExecutingLine]);
    sendUSBText(outputBuffer);
    memset(outputBuffer, 0, sizeof(outputBuffer));
    
    // Crear una copia temporal para evitar recursión
    char temp_command[MAX_LINE_LENGTH];
    strncpy(temp_command, gcodeProgram[currentExecutingLine], MAX_LINE_LENGTH - 1);
    temp_command[MAX_LINE_LENGTH - 1] = '\0';
    
    // Ejecutar el comando directamente usando el parser modular
    uint8_t status = gc_execute_line(temp_command);
    
    // Verificar resultado
    if (status == STATUS_OK) {
        // Comando ejecutado exitosamente, avanzar a la siguiente línea
        currentExecutingLine++;
        lastProgramTime = currentTime;
    } else {
        // Error en la ejecución
        sprintf(outputBuffer, "error: codigo %d en linea %d\r\n", status, currentExecutingLine + 1);
        sendUSBText(outputBuffer);
        memset(outputBuffer, 0, sizeof(outputBuffer));
        isProgramRunning = false;
    }
}

/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // // Tu código de interrupción aquí
    // static uint32_t lastInterruptTime = 0;
    // uint32_t currentTime = HAL_GetTick();
    
    // if (currentTime - lastInterruptTime < 50) return; // Debounce
    // lastInterruptTime = currentTime;
    
    // switch(GPIO_Pin) {
    //     case GPIO_PIN_12: // X_MIN_PIN
    //         if (currentX != 0) endstop_error_handler('X');
    //         break;
    //     case GPIO_PIN_13: // Y_MIN_PIN  
    //         if (currentY != 0) endstop_error_handler('Y');
    //         break;
    //     case GPIO_PIN_14: // Z_MIN_PIN
    //         if (currentZ != 0) endstop_error_handler('Z');
    //         break;
    // }
}

void endstop_error_handler(char axis)
{
    // disableSteppers();
    // sprintf(outputBuffer, "ERROR: Final de carrera %c presionado fuera de home!\r\n", axis);
    // CDC_Transmit_Queued((uint8_t*)outputBuffer, strlen(outputBuffer));
    // HAL_GPIO_WritePin(GPIOB, LED_ERROR, GPIO_PIN_SET);
}
/* USER CODE END 5 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
    // Detener todos los motores de forma segura
  disableSteppers();
  
  // Enviar mensaje de error por USB CDC
  CDC_Transmit_Queued((uint8_t*)"ERROR CRITICO DEL SISTEMA\r\n", 27);
  
  // LED de error
  HAL_GPIO_WritePin(GPIOB, LED_ERROR, GPIO_PIN_SET);
  while (1)
  {
        // Parpadear LED de error
    HAL_GPIO_TogglePin(GPIOB, LED_ERROR);
    HAL_Delay(500);
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
