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
//#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Función auxiliar para envío USB según el método configurado
void sendUSBText(const char* message) {
    //uint16_t len = strlen(message);
    //CDC_Transmit_Queued((uint8_t*)message, len);
    HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
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


void processProgram(void);  // Nueva función para ejecutar programa progresivamente


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
    // Led de encendido inicial
    HAL_GPIO_WritePin(GPIOB, LED_CHECK, GPIO_PIN_SET);  // Encender
    // Envío inicial usando cola
    CDC_Transmit_Queued((uint8_t*)"G-code listo\r\n", 14); 
    
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Procesar cola de transmisión USB CDC
    CDC_TxQueue_Process();
    
    loop();
    
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
            // El error ya fue enviado por addLineToProgram
            // Detener el modo de almacenamiento
            isStoringProgram = false;
            isProgramLoaded = false;
            programLineCount = 0;
        }
        return;
    }
    
    // Procesamiento normal de G-code
    char line_copy[100];
    strncpy(line_copy, command, sizeof(line_copy) - 1);
    line_copy[sizeof(line_copy) - 1] = '\0';
    
    uint8_t status = gc_execute_line(line_copy);
    
    
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
    
}



// Callback específico para movimiento rápido G0
void moveAxesRapidCallback(float x, float y, float z, bool x_defined, bool y_defined, bool z_defined) {
    // Usar movimiento directo (modo compatibilidad)
    float target_x = x_defined ? x : NAN;
    float target_y = y_defined ? y : NAN;
    float target_z = z_defined ? z : NAN;
    
    moveAxesWithFeedRate(target_x, target_y, target_z, rapidRate, true);
}

void moveAxesArcCallback(float x, float y, float r, bool clockwise, float feedRate, bool f_defined) {
    // Actualizar feed rate actual si se especifica
    if (f_defined && feedRate > 0) {
        currentFeedRate = feedRate;
    }
    
    // Usar movimiento directo con feed rate actual
    arc_move_r(x, y, r, clockwise, currentFeedRate);
}

// Callback específico para movimiento lineal G1 con feed rate 
void moveAxesLinearCallback(float x, float y, float z, float feedRate, bool x_defined, bool y_defined, bool z_defined, bool f_defined) {
    // Actualizar feed rate actual si se especifica
    if (f_defined && feedRate > 0) {
        currentFeedRate = feedRate;
    }
    
    float target_x = x_defined ? x : NAN;
    float target_y = y_defined ? y : NAN;
    float target_z = z_defined ? z : NAN;
    
    moveAxesWithFeedRate(target_x, target_y, target_z, currentFeedRate, false);
}

// Función para mostrar la configuración actual del sistema
void showConfiguration(void) {
    
    sendUSBText("=== CONFIGURACIÓN CNC ===\r\n");

    sprintf(outputBuffer, "Steps per mm X: %d\r\n", STEPS_PER_MM_X);
    sendUSBText(outputBuffer);
    

    sprintf(outputBuffer, "Steps per mm Y: %d\r\n", STEPS_PER_MM_Y);
    sendUSBText(outputBuffer);
    

    sprintf(outputBuffer, "Steps per mm Z: %d\r\n", STEPS_PER_MM_Z);
    sendUSBText(outputBuffer);
    

    sprintf(outputBuffer, "Step delay: %d us\r\n", STEP_DELAY_US);
    sendUSBText(outputBuffer);
    
    
    // Convertir feed rate actual a enteros
    int feed_int = (int)currentFeedRate;
    int feed_dec = (int)((currentFeedRate - feed_int) * 10);
    sprintf(outputBuffer, "Feed rate actual: %d.%d mm/min\r\n", feed_int, feed_dec);
    sendUSBText(outputBuffer);
    

    // Convertir velocidades a enteros para mostrar
    int rapid_int = (int)rapidRate;
    int rapid_dec = (int)((rapidRate - rapid_int) * 10);
    sprintf(outputBuffer, "Velocidad rápida (G0): %d.%d mm/min\r\n", rapid_int, rapid_dec);
    sendUSBText(outputBuffer);
    
    
    int max_int = (int)maxFeedRate;
    int max_dec = (int)((maxFeedRate - max_int) * 10);
    sprintf(outputBuffer, "Velocidad máxima: %d.%d mm/min\r\n", max_int, max_dec);
    sendUSBText(outputBuffer);
    

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
    HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
    
    // Enviar mensaje de inicio de homing
    sendUSBText("Iniciando secuencia de homing...\r\n");
    
    // FASE 1: Movimiento rápido hacia los finales de carrera
    sprintf(outputBuffer, "Fase 1: Buscando finales de carrera...\r\n");
    sendUSBText(outputBuffer);
    

    // Homing del eje X
    sprintf(outputBuffer, "Homing eje X...\r\n");
    sendUSBText(outputBuffer);
    

    // Mover hacia el final de carrera X (dirección negativa)
    HAL_GPIO_WritePin(GPIOB, X_DIR_PIN, GPIO_PIN_RESET); // Dirección negativa
    while (!isEndstopPressed('X')) {
        X_stepOnce();
        delay_us(STEP_DELAY_US);
    }
    
    // FASE 1: Retroceder 2mm del final de carrera X
    HAL_GPIO_WritePin(GPIOB, X_DIR_PIN, GPIO_PIN_SET); // Dirección positiva
    for (int i = 0; i < 4*STEPS_PER_MM_X; i++) { // Exactamente 2mm
        X_stepOnce();
        delay_us(STEP_DELAY_US);
    }
    
    // Verificar que se liberó el switch después de alejarse 2mm
    if (isEndstopPressed('X')) {
        sprintf(outputBuffer, "Error: Fin de carrera X no se liberó después de alejarse 2mm\r\n");
        sendUSBText(outputBuffer);
        
        return;
    }
    

    
    // FASE 2: Regresar 4mm hacia el final de carrera X
    HAL_GPIO_WritePin(GPIOB, X_DIR_PIN, GPIO_PIN_RESET); // Dirección negativa
    for (int i = 0; i < 4*STEPS_PER_MM_X; i++) { // Exactamente 4mm de regreso
        X_stepOnce();
        delay_us(STEP_DELAY_US *3); // Movimiento lento para precisión
    }
    // Pausa para estabilización
    HAL_Delay(100);
    // Verificar que el final de carrera esté presionado después de regresar 4mm
    if (!isEndstopPressed('X')) {
        sprintf(outputBuffer, "Error: Fin de carrera X no está presionado después de regresar 4mm\r\n");
        sendUSBText(outputBuffer);
        
        return;
    }
    currentX = 0; // Establecer posición home
    sprintf(outputBuffer, "Eje X en posición home\r\n");
    sendUSBText(outputBuffer);
    


    // Homing del eje Y
    sprintf(outputBuffer, "Homing eje Y...\r\n");
    sendUSBText(outputBuffer);
    

    // Mover hacia el final de carrera Y (dirección negativa)
    HAL_GPIO_WritePin(GPIOB, Y_DIR_PIN, GPIO_PIN_RESET); // Dirección negativa
    while (!isEndstopPressed('Y')) {
        Y_stepOnce();
        delay_us(STEP_DELAY_US);
    }
    
    // FASE 1: Retroceder 4mm del final de carrera Y
    HAL_GPIO_WritePin(GPIOB, Y_DIR_PIN, GPIO_PIN_SET); // Dirección positiva
    for (int i = 0; i < 4*STEPS_PER_MM_Y; i++) { // Exactamente 4mm
        Y_stepOnce();
        delay_us(STEP_DELAY_US);
    }
    
    // Verificar que se liberó el switch después de alejarse 4mm
    if (isEndstopPressed('Y')) {
        sprintf(outputBuffer, "Error: Fin de carrera Y no se liberó después de alejarse 4mm\r\n");
        sendUSBText(outputBuffer);
        
        return;
    }
    

    
    // FASE 2: Regresar 4mm hacia el final de carrera Y
    HAL_GPIO_WritePin(GPIOB, Y_DIR_PIN, GPIO_PIN_RESET); // Dirección negativa
    for (int i = 0; i < 4.02*STEPS_PER_MM_Y; i++) { // Exactamente 4mm de regreso
        Y_stepOnce();
        delay_us(STEP_DELAY_US*3); // Movimiento lento para precisión
    }
    // Pausa para estabilización
    HAL_Delay(100);
    // Verificar que el final de carrera esté presionado después de regresar 4mm
    if (!isEndstopPressed('Y')) {
        sprintf(outputBuffer, "Error: Fin de carrera Y no está presionado después de regresar 4mm\r\n");
        sendUSBText(outputBuffer);
        
        return;
    }
    
    currentY = 0; // Establecer posición home
    sprintf(outputBuffer, "Eje Y en posición home\r\n");
    sendUSBText(outputBuffer);
    

    CDC_TxQueue_Process();
    // Homing del eje Z
    sprintf(outputBuffer, "Homing eje Z...\r\n");
    sendUSBText(outputBuffer);CDC_TxQueue_Process();
    

    // Mover hacia el final de carrera Z (dirección negativa)
    HAL_GPIO_WritePin(GPIOA, Z_DIR_PIN, GPIO_PIN_SET); // Dirección negativa
    while (!isEndstopPressed('Z')) {
        Z_stepOnce();
        delay_us(STEP_DELAY_US/3);
    }
    
    // FASE 1: Retroceder 2mm del final de carrera Z
    HAL_GPIO_WritePin(GPIOA, Z_DIR_PIN, GPIO_PIN_RESET); // Dirección positiva
    for (int i = 0; i < 2*STEPS_PER_MM_Z; i++) { // Exactamente 2mm
        Z_stepOnce();
        delay_us(STEP_DELAY_US/3);
    }
    
    // Verificar que se liberó el switch después de alejarse 2mm
    if (isEndstopPressed('Z')) {
        sprintf(outputBuffer, "Error: Fin de carrera Z no se liberó después de alejarse 2mm\r\n");
        sendUSBText(outputBuffer);
        
        return;
    }
    
    
    
    // FASE 2: Regresar 2mm hacia el final de carrera Z
    HAL_GPIO_WritePin(GPIOA, Z_DIR_PIN, GPIO_PIN_SET); // Dirección negativa
    for (int i = 0; i < 2*STEPS_PER_MM_Z; i++) { // Exactamente 2mm de regreso
        Z_stepOnce();
        delay_us(STEP_DELAY_US); // Movimiento lento para precisión
    }
    // Pausa para estabilización
    HAL_Delay(100);
    // Verificar que el final de carrera esté presionado después de regresar 2mm
    if (!isEndstopPressed('Z')) {
        sprintf(outputBuffer, "Error: Fin de carrera Z no está presionado después de regresar 2mm\r\n");
        sendUSBText(outputBuffer);
        
        return;
    }
    
    currentZ = 0; // Establecer posición home
    sprintf(outputBuffer, "Eje Z en posición home\r\n");
    sendUSBText(outputBuffer);CDC_TxQueue_Process();
    

    // Mensaje final
    sprintf(outputBuffer, "Homing completado. Todos los ejes en posición home.\r\n");
    sendUSBText(outputBuffer);
    

    CDC_TxQueue_Process();

    // Rehabilitar interrupciones
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

// ========================================================================
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
        
        sendUSBText("Use 'PROGRAM_RUN' para ejecutar o 'PROGRAM_INFO' para ver detalles\r\n");
    } else {
        sendUSBText("No se almacenaron lineas\r\n");
    }
    
    sendUSBText("ok\r\n");
}

/**
  * @brief  Agrega una línea al programa almacenado
  * @param  line: Línea de G-code a agregar
  * @retval true si se agregó exitosamente, false si hay error
  */
bool addLineToProgram(const char* line) {
    if (programLineCount >= MAX_GCODE_LINES) {
        sendUSBText("error: buffer de programa lleno\r\n");
        return false; // Buffer lleno
    }
    
    // Copiar la línea, eliminando espacios al inicio y final
    const char* start = line;
    while (*start == ' ' || *start == '\t') start++; // Saltar espacios iniciales
    
    if (strlen(start) == 0) {
        return true; // Línea vacía, no la almacenamos pero no es error
    }
    
    // Crear copia para verificación de límites
    char line_copy[MAX_LINE_LENGTH];
    strncpy(line_copy, start, MAX_LINE_LENGTH - 1);
    line_copy[MAX_LINE_LENGTH - 1] = '\0';
    
    // Eliminar \r\n del final si existen
    int len = strlen(line_copy);
    while (len > 0 && (line_copy[len-1] == '\r' || line_copy[len-1] == '\n')) {
        line_copy[len-1] = '\0';
        len--;
    }
    
    // Verificar límites antes de agregar al programa
    // Solo verificamos comandos de movimiento (G0, G1, G2, G3)
    if (strncmp(line_copy, "G0", 2) == 0 || strncmp(line_copy, "G1", 2) == 0 || 
        strncmp(line_copy, "G2", 2) == 0 || strncmp(line_copy, "G3", 2) == 0) {
        
        // Parsear la línea para extraer coordenadas
        uint8_t parse_status = gc_parse_line(line_copy);
        if (parse_status != STATUS_OK) {
            char error_msg[100];
            sprintf(error_msg, "error: línea %d - ", programLineCount + 1);
            sendUSBText(error_msg);
            report_status_message(parse_status);
            return false;
        }
        
        // Verificar límites de software
        uint8_t limit_status = check_soft_limits(gc_block.values.x, gc_block.values.y, gc_block.values.z,
                                                 gc_block.values.x_defined, gc_block.values.y_defined, gc_block.values.z_defined);
        if (limit_status != STATUS_OK) {
            char error_msg[150];
            sprintf(error_msg, "error: línea %d viola límites - ", programLineCount + 1);
            sendUSBText(error_msg);
            report_status_message(limit_status);
            sendUSBText("Carga de programa cancelada\r\n");
            return false;
        }
    }
    
    // Si llegamos aquí, la línea es válida, agregarla al programa
    strncpy(gcodeProgram[programLineCount], line_copy, MAX_LINE_LENGTH - 1);
    gcodeProgram[programLineCount][MAX_LINE_LENGTH - 1] = '\0';
    
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
  * @brief  Ejecuta el programa completo almacenado
  * @retval None
  */
void runProgram(void) {

    if (!isProgramLoaded || programLineCount == 0) {
        sprintf(outputBuffer, "error: No hay programa cargado\r\n");
        sendUSBText(outputBuffer);
        
        return;
    }
    
    isProgramRunning = true;
    currentExecutingLine = 0;

    sprintf(outputBuffer, "Iniciando ejecucion del programa (%d lineas)\r\n", programLineCount);
    sendUSBText(outputBuffer);
    
    
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
        
        return;
    }
    
    if (currentExecutingLine >= programLineCount) {
        sprintf(outputBuffer, "Programa completado\r\n");
        sendUSBText(outputBuffer);
        
        isProgramRunning = false;
        return;
    }

    sprintf(outputBuffer, "Ejecutando linea %d: %s\r\n", currentExecutingLine + 1, gcodeProgram[currentExecutingLine]);
    sendUSBText(outputBuffer);
    
    // Ejecutar el comando
    char temp_command[MAX_LINE_LENGTH];
    strncpy(temp_command, gcodeProgram[currentExecutingLine], MAX_LINE_LENGTH - 1);
    temp_command[MAX_LINE_LENGTH - 1] = '\0';
    
    uint8_t status = gc_execute_line(temp_command);
    
    if (status == STATUS_OK) {
        currentExecutingLine++;
    } else {
        sprintf(outputBuffer, "error: codigo %d en linea %d\r\n", status, currentExecutingLine + 1);
        sendUSBText(outputBuffer);
        
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
    
    sprintf(outputBuffer, "ok\r\n");
    sendUSBText(outputBuffer);
    
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
    
    sprintf(outputBuffer, "ok\r\n");
    sendUSBText(outputBuffer);
    
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
    
    sendUSBText("\r\nCOMANDOS G-CODE BASICOS:\r\n");
    sendUSBText("G0 X Y Z       - Movimiento rapido\r\n");
    sendUSBText("G1 X Y Z F     - Movimiento lineal con feed rate\r\n");
    sendUSBText("G28            - Homing (ir a origen)\r\n");
    sendUSBText("G92 X Y Z      - Establecer posicion actual\r\n");
    sendUSBText("M17            - Habilitar motores\r\n");
    sendUSBText("M18 / M84      - Deshabilitar motores\r\n");
    sendUSBText("M114           - Reportar posicion actual\r\n");
    sendUSBText("M503           - Mostrar configuracion\r\n");
    sendUSBText("M505           - Mostrar limites de la maquina\r\n");
    
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
    

    sprintf(outputBuffer, "Mensajes en cola: %d/%d\r\n", CDC_TxQueue_GetCount(), 10);
    sendUSBText(outputBuffer);
    

    sprintf(outputBuffer, "Cola llena: %s\r\n", CDC_TxQueue_IsFull() ? "SI" : "NO");
    sendUSBText(outputBuffer);
    

    sprintf(outputBuffer, "Método transmisión: ");
    sendUSBText(outputBuffer);
    
    
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
  * @brief  Muestra el estado del programa en ejecución
  * @retval None
  */
void showProgramStatus(void) {
    sprintf(outputBuffer, "\r\n=== ESTADO PROGRAMA G-CODE ===\r\n");
    sendUSBText(outputBuffer);
    
    
    sprintf(outputBuffer, "Programa cargado: %s\r\n", isProgramLoaded ? "SI" : "NO");
    sendUSBText(outputBuffer);
    
    
    sprintf(outputBuffer, "Programa ejecutándose: %s\r\n", isProgramRunning ? "SI" : "NO");
    sendUSBText(outputBuffer);
    
    
    if (isProgramLoaded) {
        sprintf(outputBuffer, "Total de líneas: %d\r\n", programLineCount);
        sendUSBText(outputBuffer);
        
        
        sprintf(outputBuffer, "Línea actual: %d\r\n", currentExecutingLine + 1);
        sendUSBText(outputBuffer);
        
        
        int progress = (programLineCount > 0) ? (currentExecutingLine * 100) / programLineCount : 0;
        sprintf(outputBuffer, "Progreso: %d%%\r\n", progress);
        sendUSBText(outputBuffer);
        
        
        if (isProgramRunning && currentExecutingLine < programLineCount) {
            sprintf(outputBuffer, "Próxima línea: %s\r\n", gcodeProgram[currentExecutingLine]);
            sendUSBText(outputBuffer);
            
        }
    }
    
    sendUSBText("=============================\r\n");
}



/**
  * @brief  Procesa la ejecución del programa de manera no bloqueante
  * @retval None
  */
void processProgram(void) {
    static uint32_t lastProgramTime __attribute__((unused)) = 0;
    uint32_t currentTime = HAL_GetTick();
    
    // Verificar si hay líneas pendientes y si el buffer tiene espacio
    if (currentExecutingLine >= programLineCount) {
        // Programa completado
       
            sprintf(outputBuffer, "Programa completado exitosamente\r\n");
            sendUSBText(outputBuffer);
            
            isProgramRunning = false;
        
        return;
    }
    
    // Ejecutar siguiente línea
    sprintf(outputBuffer, "Ejecutando linea %d: %s\r\n", currentExecutingLine + 1, gcodeProgram[currentExecutingLine]);
    sendUSBText(outputBuffer);
    
    
    // Crear una copia temporal para evitar recursión
    char temp_command[MAX_LINE_LENGTH];
    strncpy(temp_command, gcodeProgram[currentExecutingLine], MAX_LINE_LENGTH - 1);
    temp_command[MAX_LINE_LENGTH - 1] = '\0';
    
    // Ejecutar el comando directamente usando el parser modular
    uint8_t status = gc_execute_line(temp_command);
    CDC_TxQueue_Process();
    // Verificar resultado
    if (status == STATUS_OK) {
        // Comando ejecutado exitosamente, avanzar a la siguiente línea
        currentExecutingLine++;
        lastProgramTime = currentTime;
    } else {
        // Error en la ejecución
        sprintf(outputBuffer, "error: codigo %d en linea %d\r\n", status, currentExecutingLine + 1);
        sendUSBText(outputBuffer);
        
        isProgramRunning = false;
    }
}

/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // Tu código de interrupción aquí
    static uint32_t lastInterruptTime = 0;
    uint32_t currentTime = HAL_GetTick();
    
    // Debounce de 50ms para evitar múltiples disparos
    if (currentTime - lastInterruptTime < 50) return;
    lastInterruptTime = currentTime;
    
    // Definir tolerancia en pasos (1mm para cada eje)
    const int32_t TOLERANCE_X = 2 * STEPS_PER_MM_X;  // 2mm en pasos
    const int32_t TOLERANCE_Y = 2 * STEPS_PER_MM_Y;  // 2mm en pasos
    const int32_t TOLERANCE_Z = 2 * STEPS_PER_MM_Z;  // 2mm en pasos

    switch(GPIO_Pin) {
        case GPIO_PIN_12: // X_MIN_PIN
            // Solo activar error si está a más de 1mm del home (posición 0)
            if (currentX > TOLERANCE_X) {
                endstop_error_handler('X');
            }
            break;
        case GPIO_PIN_13: // Y_MIN_PIN
            // Solo activar error si está a más de 1mm del home (posición 0)
            if (currentY > TOLERANCE_Y) {
                endstop_error_handler('Y');
            }
            break;
        case GPIO_PIN_14: // Z_MIN_PIN
            // Solo activar error si está a más de 1mm del home (posición 0)
            if (currentZ > TOLERANCE_Z) {
                endstop_error_handler('Z');
            }
            break;
    }
}

void endstop_error_handler(char axis)
{
    disableSteppers();
    
    // Calcular posición actual en mm para el mensaje
    float current_pos_mm;
    switch(axis) {
        case 'X':
            current_pos_mm = currentX / (float)STEPS_PER_MM_X;
            break;
        case 'Y':
            current_pos_mm = currentY / (float)STEPS_PER_MM_Y;
            break;
        case 'Z':
            current_pos_mm = currentZ / (float)STEPS_PER_MM_Z;
            break;
        default:
            current_pos_mm = 0.0f;
    }
    
    // Convertir a enteros para evitar problemas con printf float
    int pos_int = (int)current_pos_mm;
    int pos_dec = (int)((current_pos_mm - pos_int) * 100);
    
    sprintf(outputBuffer, "ERROR: Final de carrera %c activado en pos=%d.%02dmm (fuera de tolerancia)!\r\n", 
            axis, pos_int, abs(pos_dec));
    CDC_Transmit_Queued((uint8_t*)outputBuffer, strlen(outputBuffer));
    HAL_GPIO_WritePin(GPIOB, LED_ERROR, GPIO_PIN_SET);
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
