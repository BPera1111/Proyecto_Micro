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
#define STATUS_SOFT_LIMIT_ERROR 5          // Error de límite de software

#define STATUS_GCODE_UNSUPPORTED_COMMAND 20
#define STATUS_GCODE_MODAL_GROUP_VIOLATION 21
#define STATUS_GCODE_UNDEFINED_FEED_RATE 22
#define STATUS_GCODE_COMMAND_VALUE_NOT_INTEGER 23
#define STATUS_GCODE_AXIS_COMMAND_CONFLICT 24
#define STATUS_GCODE_WORD_REPEATED 25
#define STATUS_GCODE_NO_AXIS_WORDS 26
#define STATUS_GCODE_INVALID_LINE_NUMBER 27
#define STATUS_GCODE_VALUE_WORD_MISSING 28


/* Definiciones de comandos modales ----------------------------------------*/
#define MODAL_GROUP_G0 0  // [G4,G10,G28,G30,G53,G92] Non-modal
#define MODAL_GROUP_G1 1  // [G0,G1,G2,G3] Motion
#define MODAL_GROUP_M4 11 // [M0,M1,M2,M30] Stopping
#define MODAL_GROUP_M7 12 // [M3,M4,M5] Spindle turning
// #define MODAL_GROUP_M8 13 // [M7,M8,M9] Coolant control

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
    uint8_t p;              // Parámetro P (solo para G4)
    bool x_defined;         // Flags para saber qué parámetros están definidos
    bool y_defined;
    bool z_defined;
    bool f_defined;
    bool s_defined;
    bool r_defined;         // Flag para radio de arco
} gc_values_t;

// Estructura para el estado modal del parser
typedef struct {
    uint8_t motion;         // G0, G1, G2, G3, etc.
    uint8_t spindle;        // M3, M4, M5
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
uint8_t check_soft_limits(float target_x, float target_y, float target_z, bool x_defined, bool y_defined, bool z_defined);
void report_machine_limits(void);

/* Callbacks para implementar en el archivo principal ---------------------*/
void performHoming(void);

void moveAxesRapidCallback(float x, float y, float z, bool x_defined, bool y_defined, bool z_defined);
void moveAxesLinearCallback(float x, float y, float z, float feedRate, bool x_defined, bool y_defined, bool z_defined, bool f_defined);
void moveAxesArcCallback(float x, float y, float r, bool clockwise, float feedRate, bool f_defined);

// Variables externas que el parser necesita acceder
extern int32_t currentX, currentY, currentZ;

#ifdef __cplusplus
}
#endif

#endif /* __GCODE_PARSER_H */
