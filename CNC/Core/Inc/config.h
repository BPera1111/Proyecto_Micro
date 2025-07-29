/*
  config.h - Configuraciones del CNC Controller
  
  Este archivo contiene todas las definiciones y configuraciones
  principales del sistema CNC.
*/

#ifndef CONFIG_H
#define CONFIG_H

// ===========================================================================================
// CONFIGURACIÓN DE DEBUG
// ===========================================================================================
#define DEBUG_MESSAGES 0  // Cambiar a 0 para desactivar mensajes de debug

// ===========================================================================================
// CONFIGURACIÓN DE MOTORES Y MOVIMIENTO
// ===========================================================================================

// Configuración real de la máquina (valores calibrados)
#define STEPS_PER_MM_X 79      // Pasos por mm para eje X
#define STEPS_PER_MM_Y 79      // Pasos por mm para eje Y
#define STEPS_PER_MM_Z 3930    // Pasos por mm para eje Z

// Timing para motores paso a paso
#define STEP_DELAY_US 800      // Delay entre pulsos de paso en microsegundos

// ===========================================================================================
// CONFIGURACIÓN DE ARCOS Y CÍRCULOS
// ===========================================================================================

// Definiciones de segmentos para arcos
#define SEGMENTS 50
#define PI 3.14159265358979323846

// Configuración de tolerancia para rectas consecutivas
#define RADIO_SUAVIZADO 2.0
#define ANGULO_MIN 175.0

// ===========================================================================================
// CONFIGURACIÓN DE PINES STM32F103C8T6 (Blue Pill)
// ===========================================================================================

// Definiciones de pines para motores paso a paso - Eje X
#define X_STEP_PIN    GPIO_PIN_6   // PB6
#define X_DIR_PIN     GPIO_PIN_7   // PB7
#define X_EN_PIN      GPIO_PIN_8   // PA8
#define X_MIN_PIN     GPIO_PIN_12  // PB12

// Definiciones de pines para motores paso a paso - Eje Y
#define Y_STEP_PIN    GPIO_PIN_9   // PB9
#define Y_DIR_PIN     GPIO_PIN_3   // PB3
#define Y_EN_PIN      GPIO_PIN_4   // PB4
#define Y_MIN_PIN     GPIO_PIN_13  // PB13

// Definiciones de pines para motores paso a paso - Eje Z
#define Z_STEP_PIN    GPIO_PIN_8   // PA8
#define Z_DIR_PIN     GPIO_PIN_9   // PA9
#define Z_EN_PIN      GPIO_PIN_10  // PA10
#define Z_MIN_PIN     GPIO_PIN_14  // PB14

// LEDs indicadores
#define LED_ERROR     GPIO_PIN_0   // PB0
#define LED_CHECK     GPIO_PIN_1   // PB1

// ===========================================================================================
// CONFIGURACIÓN DE VELOCIDADES POR DEFECTO
// ===========================================================================================

// Velocidades por defecto (en mm/min)
#define DEFAULT_FEED_RATE 100.0f     // Feed rate por defecto
#define DEFAULT_RAPID_RATE 1000.0f   // Velocidad rápida para G0
#define DEFAULT_MAX_FEED_RATE 2000.0f // Velocidad máxima permitida

// ===========================================================================================
// CONFIGURACIÓN DE BUFFERS
// ===========================================================================================

// Buffer para mensajes de salida
#define OUTPUT_BUFFER_SIZE 200

// Buffer para comandos USB CDC
#define USB_BUFFER_SIZE 100

// Buffer para almacenar programa G-code
#define MAX_GCODE_LINES 100    // Máximo número de líneas de G-code a almacenar
#define MAX_LINE_LENGTH 80     // Longitud máxima de cada línea de G-code

// ===========================================================================================
// CONFIGURACIÓN DEL PLANNER LOOKAHEAD
// ===========================================================================================

// Buffer del planner para lookahead
#define PLANNER_BUFFER_SIZE 8           // Buffer de 8 líneas lookahead
#define MAX_VELOCITY_CHANGE 500.0f      // Cambio máximo de velocidad (mm/min)
#define JUNCTION_DEVIATION 0.1f         // Desviación permitida en juntas (mm)
#define ACCELERATION 1000.0f            // Aceleración por defecto (mm/min²)
#define MIN_SEGMENT_TIME_US 20000       // Tiempo mínimo de segmento (20ms)

// ===========================================================================================
// CÓDIGOS DE ALARMA Y ERROR
// ===========================================================================================

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

// ===========================================================================================
// CONFIGURACIÓN DE COMUNICACIÓN USB CDC
// ===========================================================================================

// Método de transmisión USB CDC (puedes cambiar esto según tus necesidades)
typedef enum {
    USB_METHOD_DIRECT,      // Envío directo (puede perderse si está ocupado)
    USB_METHOD_RETRY,       // Envío con reintentos (bloquea hasta enviar)
    USB_METHOD_QUEUED       // Envío mediante cola (recomendado)
} USBTransmitMethod_t;

#define USB_TRANSMIT_METHOD USB_METHOD_QUEUED  // ← Cambiar aquí el método

#endif // CONFIG_H
