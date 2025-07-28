/**
  ******************************************************************************
  * @file           : gcode_parser.h
  * @brief          : Parser G-code mejorado basado en GRBL32
  ******************************************************************************
  * @attention
  *
  * Parser G-code profesional compatible con GRBL para STM32F103
  * Implementa validación de grupos modales y códigos de error estándar
  *
  ******************************************************************************
  */

#ifndef __GCODE_PARSER_H
#define __GCODE_PARSER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* Códigos de error estándar GRBL ------------------------------------------*/
#define STATUS_OK 0
#define STATUS_EXPECTED_COMMAND_LETTER 1
#define STATUS_BAD_NUMBER_FORMAT 2
#define STATUS_INVALID_STATEMENT 3
#define STATUS_NEGATIVE_VALUE 4
#define STATUS_SETTING_DISABLED 5
#define STATUS_SETTING_STEP_PULSE_MIN 6
#define STATUS_SETTING_READ_FAIL 7
#define STATUS_IDLE_ERROR 8
#define STATUS_SYSTEM_GC_LOCK 9
#define STATUS_SOFT_LIMIT_ERROR 10
#define STATUS_OVERFLOW 11
#define STATUS_MAX_STEP_RATE_EXCEEDED 12
#define STATUS_CHECK_DOOR 13
#define STATUS_LINE_LENGTH_EXCEEDED 14
#define STATUS_TRAVEL_EXCEEDED 15
#define STATUS_INVALID_JOG_COMMAND 16
#define STATUS_SETTING_DISABLED_LASER 17
#define STATUS_GCODE_UNSUPPORTED_COMMAND 20
#define STATUS_GCODE_MODAL_GROUP_VIOLATION 21
#define STATUS_GCODE_UNDEFINED_FEED_RATE 22
#define STATUS_GCODE_COMMAND_VALUE_NOT_INTEGER 23
#define STATUS_GCODE_AXIS_COMMAND_CONFLICT 24
#define STATUS_GCODE_WORD_REPEATED 25
#define STATUS_GCODE_NO_AXIS_WORDS 26
#define STATUS_GCODE_INVALID_LINE_NUMBER 27
#define STATUS_GCODE_VALUE_WORD_MISSING 28
#define STATUS_GCODE_UNSUPPORTED_COORD_SYS 29
#define STATUS_GCODE_G53_INVALID_MOTION_MODE 30
#define STATUS_GCODE_AXIS_WORDS_EXIST 31
#define STATUS_GCODE_NO_AXIS_WORDS_IN_PLANE 32
#define STATUS_GCODE_INVALID_TARGET 33
#define STATUS_GCODE_ARC_RADIUS_ERROR 34
#define STATUS_GCODE_NO_OFFSETS_IN_PLANE 35
#define STATUS_GCODE_UNUSED_WORDS 36
#define STATUS_GCODE_G43_DYNAMIC_AXIS_ERROR 37
#define STATUS_GCODE_MAX_VALUE_EXCEEDED 38

/* Definiciones de comandos modales ----------------------------------------*/
#define MODAL_GROUP_G0 0  // [G4,G10,G28,G30,G53,G92] Non-modal
#define MODAL_GROUP_G1 1  // [G0,G1,G2,G3] Motion
#define MODAL_GROUP_G2 2  // [G17,G18,G19] Plane selection
#define MODAL_GROUP_G3 3  // [G90,G91] Distance mode
#define MODAL_GROUP_G5 5  // [G93,G94] Feed rate mode
#define MODAL_GROUP_G6 6  // [G20,G21] Units
#define MODAL_GROUP_G12 9 // [G54,G55,G56,G57,G58,G59] Coordinate system
#define MODAL_GROUP_M4 11 // [M0,M1,M2,M30] Stopping
#define MODAL_GROUP_M7 12 // [M3,M4,M5] Spindle turning
#define MODAL_GROUP_M8 13 // [M7,M8,M9] Coolant control

/* Valores de comandos -----------------------------------------------------*/
#define MOTION_MODE_SEEK 0     // G0
#define MOTION_MODE_LINEAR 1   // G1
#define MOTION_MODE_CW_ARC 2   // G2  
#define MOTION_MODE_CCW_ARC 3  // G3

/* Máscaras de bits para tracking ------------------------------------------*/
#define bit(n) (1<<n)
#define bit_istrue(x,mask) ((x & mask) != 0)
#define bit_isfalse(x,mask) ((x & mask) == 0)
#define bit_true(x,mask) (x |= mask)
#define bit_false(x,mask) (x &= ~mask)

/* Estructuras de datos ----------------------------------------------------*/

// Estructura para almacenar valores de una línea G-code
typedef struct {
    float x, y, z;          // Coordenadas de destino
    float f;                // Feed rate
    float s;                // Spindle speed
    float r;                // Radio para arcos (G2/G3)
    int32_t n;              // Line number
    uint8_t p, l;           // Parámetros P y L
    bool x_defined;         // Flags para saber qué parámetros están definidos
    bool y_defined;
    bool z_defined;
    bool f_defined;
    bool s_defined;
    bool r_defined;         // Nuevo flag para radio de arco
} gc_values_t;

// Estructura para el estado modal del parser
typedef struct {
    uint8_t motion;         // G0, G1, G2, G3, etc.
    uint8_t coord_select;   // G54, G55, G56, etc.
    uint8_t plane_select;   // G17, G18, G19
    uint8_t units;          // G20, G21
    uint8_t distance;       // G90, G91
    uint8_t feed_rate;      // G93, G94
    uint8_t spindle;        // M3, M4, M5
    uint8_t coolant;        // M7, M8, M9
    uint8_t program_flow;   // M0, M1, M2, M30
} gc_modal_t;

// Estructura principal del parser
typedef struct {
    gc_modal_t modal;       // Estados modales actuales
    gc_values_t values;     // Valores de la línea actual
    uint8_t non_modal_command; // Comando no modal (G4, G28, etc.)
} gc_block_t;

/* Variables globales exportadas -------------------------------------------*/
extern gc_block_t gc_block;              // Bloque de parsing actual
extern gc_modal_t gc_state_modal;        // Estado modal persistente

/* Prototipos de funciones -------------------------------------------------*/
void gc_init(void);
uint8_t gc_execute_line(char *line);
uint8_t gc_parse_line(char *line);
uint8_t gc_execute_block(void);
void gc_clear_block(void);
bool read_float(char *line, uint8_t *char_counter, float *float_ptr);
void report_status_message(uint8_t status_code);

/* Callbacks para implementar en el archivo principal ---------------------*/
// Estas funciones deben ser implementadas en main.c para el control del hardware
void moveAxes(float x, float y, float z);
void performHoming(void);
void showConfiguration(void);
void sendUSBMessage(const uint8_t* data, uint16_t length);

// Variables externas que el parser necesita acceder
extern int32_t currentX, currentY, currentZ;

// Constantes de configuración de hardware
extern const uint16_t STEPS_PER_MM_X;
extern const uint16_t STEPS_PER_MM_Y;
extern const uint16_t STEPS_PER_MM_Z;

#ifdef __cplusplus
}
#endif

#endif /* __GCODE_PARSER_H */
