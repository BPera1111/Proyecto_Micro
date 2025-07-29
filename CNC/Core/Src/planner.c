/**
  ******************************************************************************
  * @file           : planner.c
  * @brief          : Implementación del Motion Planner con Lookahead
  ******************************************************************************
  * @attention
  *
  * Planner de movimientos con buffer lookahead para suavizado de trayectorias
  * y optimización de velocidades. Basado en algoritmos de GRBL y Marlin.
  *
  ******************************************************************************
  */

#include "planner.h"
#include "motion.h"
#include "main.h"
#include <string.h>

/* Variables globales del Planner --------------------------------------------*/
planner_block_t planner_buffer[PLANNER_BUFFER_SIZE];
volatile uint8_t planner_buffer_head = 0;
volatile uint8_t planner_buffer_tail = 0;
volatile uint8_t planner_buffer_count = 0;
planner_state_t planner_state;

/* Funciones privadas --------------------------------------------------------*/
static void planner_reverse_pass(void);
static void planner_forward_pass(void);
static bool planner_execute_block(planner_block_t *block);
static void planner_calculate_trapezoid(planner_block_t *block, float entry_factor, float exit_factor);
static uint8_t planner_next_block_index(uint8_t block_index);
static uint8_t planner_prev_block_index(uint8_t block_index);

/**
 * @brief Inicializa el planner
 */
void planner_init(void) {
    // Limpiar buffer
    memset(planner_buffer, 0, sizeof(planner_buffer));
    
    // Resetear índices
    planner_buffer_head = 0;
    planner_buffer_tail = 0;
    planner_buffer_count = 0;
    
    // Inicializar estado
    memset(&planner_state, 0, sizeof(planner_state));
    planner_state.is_running = false;
    planner_state.buffer_full = false;
    planner_state.total_blocks_processed = 0;
    
    // Obtener posición actual del sistema de motion
    float current_pos[3];
    getCurrentPositionMM(&current_pos[0], &current_pos[1], &current_pos[2]);
    planner_set_current_position(current_pos);
    
    #if DEBUG_MESSAGES
    sendUSBText("[PLANNER] Inicializado correctamente\r\n");
    #endif
}

/**
 * @brief Reinicia el planner y limpia el buffer
 */
void planner_reset(void) {
    planner_init();
    #if DEBUG_MESSAGES
    sendUSBText("[PLANNER] Reset completado\r\n");
    #endif
}

/**
 * @brief Agrega un movimiento lineal al buffer (G0, G1)
 */
bool planner_buffer_line(float target[3], float feed_rate, bool is_rapid) {
    // Verificar si hay espacio en el buffer
    if (planner_buffer_count >= PLANNER_BUFFER_SIZE) {
        planner_state.buffer_full = true;
        return false;
    }
    
    // Obtener bloque libre
    planner_block_t *block = &planner_buffer[planner_buffer_head];
    
    // Configurar tipo de movimiento
    block->move_type = is_rapid ? MOVE_TYPE_RAPID : MOVE_TYPE_LINEAR;
    
    // Copiar posiciones
    memcpy(block->start_pos, planner_state.position, sizeof(float) * 3);
    memcpy(block->target_pos, target, sizeof(float) * 3);
    
    // Calcular distancia
    block->distance = planner_calculate_distance(block->start_pos, block->target_pos);
    
    // Si la distancia es muy pequeña, ignorar el movimiento
    if (block->distance < 0.001f) {
        return true; // No error, pero tampoco agregar
    }
    
    // Configurar velocidades
    block->nominal_speed = is_rapid ? rapidRate : feed_rate;
    if (block->nominal_speed > maxFeedRate) {
        block->nominal_speed = maxFeedRate;
    }
    
    // Configurar aceleración
    block->acceleration = ACCELERATION;
    
    // Inicializar velocidades
    block->entry_speed = 0.0f;
    block->exit_speed = 0.0f;
    block->max_entry_speed = block->nominal_speed;
    
    // Flags de estado
    block->busy = true;
    block->executed = false;
    block->recalculate_flag = true;
    block->nominal_length_flag = true;
    
    // Actualizar posición del planner
    memcpy(planner_state.position, target, sizeof(float) * 3);
    
    // Avanzar índice de cabeza
    planner_buffer_head = planner_next_block_index(planner_buffer_head);
    planner_buffer_count++;
    
    // Recalcular velocidades
    planner_recalculate();
    
    #if DEBUG_MESSAGES
    char debug_msg[150];
    sprintf(debug_msg, "[PLANNER] Línea agregada: (%.2f,%.2f,%.2f) -> (%.2f,%.2f,%.2f), dist=%.3f, vel=%.1f\r\n",
           block->start_pos[0], block->start_pos[1], block->start_pos[2],
           block->target_pos[0], block->target_pos[1], block->target_pos[2],
           block->distance, block->nominal_speed);
    sendUSBText(debug_msg);
    #endif
    
    return true;
}

/**
 * @brief Agrega un movimiento de arco al buffer (G2, G3)
 */
bool planner_buffer_arc(float target[3], float offset[2], bool is_clockwise, 
                       bool is_radius_mode, float feed_rate) {
    // Verificar si hay espacio en el buffer
    if (planner_buffer_count >= PLANNER_BUFFER_SIZE) {
        planner_state.buffer_full = true;
        return false;
    }
    
    // Obtener bloque libre
    planner_block_t *block = &planner_buffer[planner_buffer_head];
    
    // Configurar tipo de movimiento
    block->move_type = is_clockwise ? MOVE_TYPE_ARC_CW : MOVE_TYPE_ARC_CCW;
    
    // Copiar posiciones
    memcpy(block->start_pos, planner_state.position, sizeof(float) * 3);
    memcpy(block->target_pos, target, sizeof(float) * 3);
    
    // Configurar parámetros de arco
    block->arc_radius_mode = is_radius_mode;
    
    if (is_radius_mode) {
        // Modo R - offset[0] contiene el radio
        block->arc_radius = offset[0];
        
        // Calcular centro del arco basado en radio
        float dx = block->target_pos[0] - block->start_pos[0];
        float dy = block->target_pos[1] - block->start_pos[1];
        float chord_distance = sqrtf(dx*dx + dy*dy);
        
        // Verificar que el radio sea válido
        if (block->arc_radius < chord_distance / 2.0f) {
            return false; // Radio muy pequeño
        }
        
        // Calcular altura del triángulo
        float h = sqrtf(block->arc_radius * block->arc_radius - 
                       (chord_distance / 2.0f) * (chord_distance / 2.0f));
        
        // Punto medio de la cuerda
        float mid_x = (block->start_pos[0] + block->target_pos[0]) / 2.0f;
        float mid_y = (block->start_pos[1] + block->target_pos[1]) / 2.0f;
        
        // Vector perpendicular unitario
        float perp_x = -dy / chord_distance;
        float perp_y = dx / chord_distance;
        
        // Centro del arco (elegir lado basado en dirección y signo del radio)
        float sign = (is_clockwise == (block->arc_radius > 0)) ? -1.0f : 1.0f;
        block->arc_center[0] = mid_x + sign * h * perp_x;
        block->arc_center[1] = mid_y + sign * h * perp_y;
        
        block->arc_radius = fabsf(block->arc_radius);
    } else {
        // Modo I,J - offset contiene las coordenadas del centro relativas
        block->arc_center[0] = block->start_pos[0] + offset[0];
        block->arc_center[1] = block->start_pos[1] + offset[1];
        
        // Calcular radio desde el centro
        float dx_start = block->start_pos[0] - block->arc_center[0];
        float dy_start = block->start_pos[1] - block->arc_center[1];
        block->arc_radius = sqrtf(dx_start*dx_start + dy_start*dy_start);
    }
    
    // Calcular longitud del arco
    float dx_start = block->start_pos[0] - block->arc_center[0];
    float dy_start = block->start_pos[1] - block->arc_center[1];
    float dx_end = block->target_pos[0] - block->arc_center[0];
    float dy_end = block->target_pos[1] - block->arc_center[1];
    
    float start_angle = atan2f(dy_start, dx_start);
    float end_angle = atan2f(dy_end, dx_end);
    
    float angular_travel = end_angle - start_angle;
    
    // Ajustar ángulo según dirección
    if (is_clockwise) {
        if (angular_travel >= 0) {
            angular_travel -= 2.0f * PI;
        }
    } else {
        if (angular_travel <= 0) {
            angular_travel += 2.0f * PI;
        }
    }
    
    block->distance = fabsf(angular_travel) * block->arc_radius;
    
    // Incluir movimiento en Z si existe
    float dz = block->target_pos[2] - block->start_pos[2];
    if (fabsf(dz) > 0.001f) {
        float linear_distance = sqrtf(block->distance * block->distance + dz * dz);
        block->distance = linear_distance;
    }
    
    // Configurar velocidades
    block->nominal_speed = feed_rate;
    if (block->nominal_speed > maxFeedRate) {
        block->nominal_speed = maxFeedRate;
    }
    
    // Configurar aceleración (menor para arcos)
    block->acceleration = ACCELERATION * 0.8f; // 80% de aceleración para arcos
    
    // Inicializar velocidades
    block->entry_speed = 0.0f;
    block->exit_speed = 0.0f;
    block->max_entry_speed = block->nominal_speed;
    
    // Flags de estado
    block->busy = true;
    block->executed = false;
    block->recalculate_flag = true;
    block->nominal_length_flag = true;
    
    // Actualizar posición del planner
    memcpy(planner_state.position, target, sizeof(float) * 3);
    
    // Avanzar índice de cabeza
    planner_buffer_head = planner_next_block_index(planner_buffer_head);
    planner_buffer_count++;
    
    // Recalcular velocidades
    planner_recalculate();
    
    #if DEBUG_MESSAGES
    char debug_msg[200];
    sprintf(debug_msg, "[PLANNER] Arco agregado: R=%.3f, centro=(%.2f,%.2f), dist=%.3f, vel=%.1f, %s\r\n",
           block->arc_radius, block->arc_center[0], block->arc_center[1],
           block->distance, block->nominal_speed, is_clockwise ? "CW" : "CCW");
    sendUSBText(debug_msg);
    #endif
    
    return true;
}

/**
 * @brief Procesa el siguiente bloque del buffer
 */
bool planner_process_next_block(void) {
    // Verificar si hay bloques pendientes
    if (planner_buffer_count == 0) {
        planner_state.is_running = false;
        return false;
    }
    
    // Obtener siguiente bloque
    planner_block_t *block = &planner_buffer[planner_buffer_tail];
    
    if (!block->busy || block->executed) {
        return false;
    }
    
    planner_state.is_running = true;
    
    // Ejecutar el bloque
    bool success = planner_execute_block(block);
    
    if (success) {
        // Marcar como ejecutado
        block->executed = true;
        block->busy = false;
        
        // Avanzar cola
        planner_buffer_tail = planner_next_block_index(planner_buffer_tail);
        planner_buffer_count--;
        planner_state.total_blocks_processed++;
        
        // Reset flag de buffer lleno
        planner_state.buffer_full = false;
        
        #if DEBUG_MESSAGES
        char debug_msg[100];
        sprintf(debug_msg, "[PLANNER] Bloque ejecutado. Buffer: %d/%d\r\n", 
               planner_buffer_count, PLANNER_BUFFER_SIZE);
        sendUSBText(debug_msg);
        #endif
    }
    
    return success;
}

/**
 * @brief Ejecuta un bloque específico
 */
static bool planner_execute_block(planner_block_t *block) {
    switch (block->move_type) {
        case MOVE_TYPE_RAPID:
        case MOVE_TYPE_LINEAR:
            // Movimiento lineal con velocidad optimizada
            moveAxesWithFeedRate(block->target_pos[0], block->target_pos[1], 
                               block->target_pos[2], block->entry_speed, 
                               block->move_type == MOVE_TYPE_RAPID);
            break;
            
        case MOVE_TYPE_ARC_CW:
            // Arco horario
            arc_move_r(block->target_pos[0], block->target_pos[1], 
                      block->arc_radius, 1);
            break;
            
        case MOVE_TYPE_ARC_CCW:
            // Arco antihorario
            arc_move_r(block->target_pos[0], block->target_pos[1], 
                      block->arc_radius, 0);
            break;
            
        default:
            return false;
    }
    
    return true;
}

/**
 * @brief Recalcula las velocidades de todo el buffer usando lookahead
 */
void planner_recalculate(void) {
    if (planner_buffer_count < 2) {
        return; // No hay suficientes bloques para optimizar
    }
    
    // Paso hacia atrás (reverse pass)
    planner_reverse_pass();
    
    // Paso hacia adelante (forward pass)
    planner_forward_pass();
}

/**
 * @brief Paso hacia atrás del algoritmo lookahead
 */
static void planner_reverse_pass(void) {
    uint8_t block_index = planner_buffer_head;
    
    // Empezar desde el último bloque agregado
    if (planner_buffer_count > 0) {
        block_index = planner_prev_block_index(block_index);
    }
    
    planner_block_t *block[3] = {NULL, NULL, NULL};
    
    // Iterar hacia atrás por el buffer
    while (block_index != planner_buffer_tail) {
        block[2] = block[1];
        block[1] = block[0];
        block[0] = &planner_buffer[block_index];
        
        if (block[1] != NULL && block[1]->recalculate_flag) {
            // Calcular velocidad de junta
            float junction_speed = 0.0f;
            
            if (block[2] != NULL) {
                junction_speed = planner_junction_velocity(block[1], block[2]);
            }
            
            // Limitar velocidad de salida
            if (junction_speed < block[1]->exit_speed) {
                block[1]->exit_speed = junction_speed;
                block[1]->recalculate_flag = false;
            }
        }
        
        block_index = planner_prev_block_index(block_index);
    }
}

/**
 * @brief Paso hacia adelante del algoritmo lookahead
 */
static void planner_forward_pass(void) {
    uint8_t block_index = planner_buffer_tail;
    planner_block_t *block[2] = {NULL, NULL};
    
    // Iterar hacia adelante por el buffer
    while (block_index != planner_buffer_head) {
        block[0] = block[1];
        block[1] = &planner_buffer[block_index];
        
        if (block[0] != NULL) {
            // Calcular velocidad máxima alcanzable
            float max_entry_speed = planner_max_allowable_speed(
                block[1]->acceleration, block[0]->exit_speed, block[1]->distance);
            
            if (block[1]->entry_speed > max_entry_speed) {
                block[1]->entry_speed = max_entry_speed;
            }
            
            // Calcular perfil de velocidad trapezoidal
            float entry_factor = block[1]->entry_speed / block[1]->nominal_speed;
            float exit_factor = block[1]->exit_speed / block[1]->nominal_speed;
            planner_calculate_trapezoid(block[1], entry_factor, exit_factor);
        }
        
        block_index = planner_next_block_index(block_index);
    }
}

/**
 * @brief Calcula el perfil de velocidad trapezoidal para un bloque
 */
static void planner_calculate_trapezoid(planner_block_t *block, float entry_factor, float exit_factor) {
    // Limitar factores entre 0 y 1
    if (entry_factor > 1.0f) entry_factor = 1.0f;
    if (exit_factor > 1.0f) exit_factor = 1.0f;
    if (entry_factor < 0.0f) entry_factor = 0.0f;
    if (exit_factor < 0.0f) exit_factor = 0.0f;
    
    // Calcular velocidades reales
    block->entry_speed = entry_factor * block->nominal_speed;
    block->exit_speed = exit_factor * block->nominal_speed;
    
    // El perfil detallado se calculará durante la ejecución
}

/**
 * @brief Calcula la velocidad de junta entre dos bloques
 */
float planner_junction_velocity(planner_block_t *before, planner_block_t *after) {
    if (before == NULL || after == NULL) {
        return 0.0f;
    }
    
    // Calcular vectores de dirección
    float before_unit[3], after_unit[3];
    
    for (int i = 0; i < 3; i++) {
        before_unit[i] = (before->target_pos[i] - before->start_pos[i]) / before->distance;
        after_unit[i] = (after->target_pos[i] - after->start_pos[i]) / after->distance;
    }
    
    // Calcular ángulo entre vectores
    float dot_product = 0.0f;
    for (int i = 0; i < 3; i++) {
        dot_product += before_unit[i] * after_unit[i];
    }
    
    // Limitar producto punto
    if (dot_product > 1.0f) dot_product = 1.0f;
    if (dot_product < -1.0f) dot_product = -1.0f;
    
    // Calcular desviación de junta
    float cos_theta = -dot_product; // Ángulo entre vectores opuestos
    float sin_half_theta = sqrtf((1.0f - cos_theta) / 2.0f);
    
    if (sin_half_theta < 0.001f) {
        // Líneas casi paralelas, velocidad máxima
        return fminf(before->nominal_speed, after->nominal_speed);
    }
    
    // Calcular velocidad máxima basada en desviación permitida
    float junction_speed = sqrtf(JUNCTION_DEVIATION * before->acceleration / sin_half_theta);
    
    // Limitar a la velocidad nominal más baja
    float max_speed = fminf(before->nominal_speed, after->nominal_speed);
    return fminf(junction_speed, max_speed);
}

/**
 * @brief Calcula la velocidad máxima alcanzable dada una distancia y aceleración
 */
float planner_max_allowable_speed(float acceleration, float target_velocity, float distance) {
    // v² = u² + 2as
    float max_speed_sq = target_velocity * target_velocity + 2.0f * acceleration * distance;
    
    if (max_speed_sq <= 0.0f) {
        return 0.0f;
    }
    
    return sqrtf(max_speed_sq);
}

/**
 * @brief Calcula la distancia entre dos puntos 3D
 */
float planner_calculate_distance(float from[3], float to[3]) {
    float dx = to[0] - from[0];
    float dy = to[1] - from[1];
    float dz = to[2] - from[2];
    
    return sqrtf(dx*dx + dy*dy + dz*dz);
}

/**
 * @brief Obtiene el siguiente índice en el buffer circular
 */
static uint8_t planner_next_block_index(uint8_t block_index) {
    return (block_index + 1) % PLANNER_BUFFER_SIZE;
}

/**
 * @brief Obtiene el índice anterior en el buffer circular
 */
static uint8_t planner_prev_block_index(uint8_t block_index) {
    return (block_index == 0) ? (PLANNER_BUFFER_SIZE - 1) : (block_index - 1);
}

/* Funciones de interfaz pública ---------------------------------------------*/

bool planner_buffer_has_space(void) {
    return planner_buffer_count < PLANNER_BUFFER_SIZE;
}

uint8_t planner_get_buffer_count(void) {
    return planner_buffer_count;
}

void planner_synchronize(void) {
    while (planner_buffer_count > 0) {
        planner_process_next_block();
        HAL_Delay(1); // Pequeña pausa para no saturar el procesador
    }
}

void planner_get_current_position(float position[3]) {
    memcpy(position, planner_state.position, sizeof(float) * 3);
}

void planner_set_current_position(float position[3]) {
    memcpy(planner_state.position, position, sizeof(float) * 3);
}

void planner_get_statistics(uint8_t *buffer_usage, uint32_t *blocks_processed, bool *is_running) {
    *buffer_usage = (planner_buffer_count * 100) / PLANNER_BUFFER_SIZE;
    *blocks_processed = planner_state.total_blocks_processed;
    *is_running = planner_state.is_running;
}
