/**
  ******************************************************************************
  * @file           : planner.h
  * @brief          : Motion Planner con Lookahead para CNC Controller
  ******************************************************************************
  * @attention
  *
  * Planner de movimientos con buffer lookahead para suavizado de trayectorias
  * y optimización de velocidades. Compatible con G0, G1, G2, G3.
  *
  ******************************************************************************
  */

#ifndef __PLANNER_H
#define __PLANNER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "config.h"
#include "gcode_parser.h"

/* Definiciones del Planner -------------------------------------------------*/
#define PLANNER_BUFFER_SIZE 8           // Buffer de 8 líneas lookahead
#define MAX_VELOCITY_CHANGE 500.0f      // Cambio máximo de velocidad (mm/min)
#define JUNCTION_DEVIATION 0.1f         // Desviación permitida en juntas (mm)
#define ACCELERATION 1000.0f            // Aceleración por defecto (mm/min²)
#define MIN_SEGMENT_TIME_US 20000       // Tiempo mínimo de segmento (20ms)

/* Tipos de movimiento -------------------------------------------------------*/
typedef enum {
    MOVE_TYPE_RAPID = 0,    // G0 - Movimiento rápido
    MOVE_TYPE_LINEAR = 1,   // G1 - Movimiento lineal
    MOVE_TYPE_ARC_CW = 2,   // G2 - Arco horario
    MOVE_TYPE_ARC_CCW = 3   // G3 - Arco antihorario
} move_type_t;

/* Estructura de bloque de movimiento ----------------------------------------*/
typedef struct {
    // Tipo de movimiento
    move_type_t move_type;
    
    // Posiciones de inicio y destino (en mm)
    float start_pos[3];     // [X, Y, Z] posición inicial
    float target_pos[3];    // [X, Y, Z] posición objetivo
    
    // Parámetros específicos para arcos (G2/G3)
    float arc_center[2];    // [I, J] centro del arco (solo X, Y)
    float arc_radius;       // Radio del arco
    bool arc_radius_mode;   // true si usa modo R, false si usa I,J
    
    // Parámetros de movimiento
    float distance;         // Distancia total del movimiento (mm)
    float nominal_speed;    // Velocidad nominal solicitada (mm/min)
    float entry_speed;      // Velocidad de entrada calculada (mm/min)
    float exit_speed;       // Velocidad de salida calculada (mm/min)
    float max_entry_speed;  // Velocidad máxima permitida en entrada
    
    // Control de aceleración
    float acceleration;     // Aceleración aplicable a este movimiento
    bool recalculate_flag;  // Flag para recalcular velocidades
    bool nominal_length_flag; // Flag indica si puede alcanzar velocidad nominal
    
    // Estado del bloque
    bool busy;              // true si el bloque está en uso
    bool executed;          // true si ya fue ejecutado
} planner_block_t;

/* Variables del Planner -----------------------------------------------------*/
extern planner_block_t planner_buffer[PLANNER_BUFFER_SIZE];
extern volatile uint8_t planner_buffer_head;    // Índice de escritura
extern volatile uint8_t planner_buffer_tail;    // Índice de lectura
extern volatile uint8_t planner_buffer_count;   // Número de bloques en buffer

/* Estado del Planner --------------------------------------------------------*/
typedef struct {
    float position[3];          // Posición actual del planner (mm)
    float previous_speed[3];    // Velocidad previa por eje
    float previous_unit_vec[3]; // Vector unitario previo
    bool is_running;            // true si el planner está ejecutando
    bool buffer_full;           // true si el buffer está lleno
    uint32_t total_blocks_processed; // Contador de bloques procesados
} planner_state_t;

extern planner_state_t planner_state;

/* Prototipos de funciones ---------------------------------------------------*/

/**
 * @brief Inicializa el planner
 */
void planner_init(void);

/**
 * @brief Reinicia el planner y limpia el buffer
 */
void planner_reset(void);

/**
 * @brief Agrega un movimiento lineal al buffer (G0, G1)
 * @param target: Posición objetivo [X, Y, Z] en mm
 * @param feed_rate: Velocidad de alimentación en mm/min
 * @param is_rapid: true para movimiento rápido (G0)
 * @return true si se agregó exitosamente, false si buffer lleno
 */
bool planner_buffer_line(float target[3], float feed_rate, bool is_rapid);

/**
 * @brief Agrega un movimiento de arco al buffer (G2, G3)
 * @param target: Posición objetivo [X, Y, Z] en mm
 * @param offset: Offset del centro [I, J] o radio R
 * @param is_clockwise: true para G2 (horario), false para G3 (antihorario)
 * @param is_radius_mode: true si usa modo R, false si usa I,J
 * @param feed_rate: Velocidad de alimentación en mm/min
 * @return true si se agregó exitosamente, false si buffer lleno
 */
bool planner_buffer_arc(float target[3], float offset[2], bool is_clockwise, 
                       bool is_radius_mode, float feed_rate);

/**
 * @brief Procesa el siguiente bloque del buffer (llamar desde loop principal)
 * @return true si se procesó un bloque, false si buffer vacío
 */
bool planner_process_next_block(void);

/**
 * @brief Recalcula las velocidades de todo el buffer usando lookahead
 */
void planner_recalculate(void);

/**
 * @brief Verifica si el buffer tiene espacio disponible
 * @return true si hay espacio, false si está lleno
 */
bool planner_buffer_has_space(void);

/**
 * @brief Obtiene el número de bloques en el buffer
 * @return Número de bloques pendientes
 */
uint8_t planner_get_buffer_count(void);

/**
 * @brief Espera hasta que el buffer esté vacío
 */
void planner_synchronize(void);

/**
 * @brief Obtiene la posición actual del planner
 * @param position: Array para almacenar [X, Y, Z]
 */
void planner_get_current_position(float position[3]);

/**
 * @brief Establece la posición actual del planner
 * @param position: Nueva posición [X, Y, Z]
 */
void planner_set_current_position(float position[3]);

/**
 * @brief Obtiene estadísticas del planner para diagnóstico
 * @param buffer_usage: Porcentaje de uso del buffer (0-100)
 * @param blocks_processed: Total de bloques procesados
 * @param is_running: Estado de ejecución
 */
void planner_get_statistics(uint8_t *buffer_usage, uint32_t *blocks_processed, bool *is_running);

/* Funciones auxiliares ------------------------------------------------------*/

/**
 * @brief Calcula la distancia entre dos puntos 3D
 */
float planner_calculate_distance(float from[3], float to[3]);

/**
 * @brief Calcula la velocidad máxima permitida en una junta
 */
float planner_max_allowable_speed(float acceleration, float target_velocity, float distance);

/**
 * @brief Calcula la velocidad de junta entre dos segmentos
 */
float planner_junction_velocity(planner_block_t *before, planner_block_t *after);

/**
 * @brief Convierte coordenadas de arco en segmentos lineales
 */
uint8_t planner_arc_to_segments(float target[3], float center[2], float radius, 
                               bool is_clockwise, float segments[][3], uint8_t max_segments);

#ifdef __cplusplus
}
#endif

#endif /* __PLANNER_H */
