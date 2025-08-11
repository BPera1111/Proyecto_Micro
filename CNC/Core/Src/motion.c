/*
  motion.c - Implementación del módulo de control de movimiento para CNC Controller
  
  Este archivo contiene todas las funciones relacionadas con el movimiento
  de los motores paso a paso y control de ejes.
*/

#include <stdlib.h>   // Para abs()
#include <stdio.h>    // Para snprintf(), sscanf(), printf()
#include <string.h>   // Para memset(), strncmp(), strcpy(), strlen()
#include <stdint.h>   // Para uint32_t
#include <stdbool.h>  // Para bool
#include <math.h>     // Para funciones matemáticas
#include "stm32f1xx_hal.h"  // Para HAL functions
#include "main.h"
#include "config.h"
#include "motion.h"


// ===========================================================================================

// VARIABLES GLOBALES DE POSICIÓN (declaradas como extern - definidas en main.c)
// ===========================================================================================

// Variables de posición actual en pasos (definidas en main.c)
extern int32_t currentX, currentY, currentZ;

// Variables de velocidad (definidas en main.c)
extern float currentFeedRate;
extern float rapidRate;
extern float maxFeedRate;
extern char gcodeProgram[MAX_GCODE_LINES][MAX_LINE_LENGTH];

// ===========================================================================================
// FUNCIONES AUXILIARES PRIVADAS
// ===========================================================================================

/**
  * @brief  Función de delay de microsegundos usando DWT
  * @param  us: Cantidad de microsegundos a esperar
  * @retval None
  */
void delay_us(uint32_t us) {
    uint32_t cycles = (SystemCoreClock / 1000000L) * us;
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles);
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

// ===========================================================================================
// FUNCIONES DE MOVIMIENTO BÁSICO POR PASOS
// ===========================================================================================

/**
  * @brief  Ejecuta un solo paso en el eje X
  * @retval None
  */
void X_stepOnce(void) {
    HAL_GPIO_WritePin(GPIOB, X_STEP_PIN, GPIO_PIN_SET);
    delay_us(2);
    HAL_GPIO_WritePin(GPIOB, X_STEP_PIN, GPIO_PIN_RESET);
}

/**
  * @brief  Ejecuta un solo paso en el eje Y
  * @retval None
  */
void Y_stepOnce(void) {
    HAL_GPIO_WritePin(GPIOB, Y_STEP_PIN, GPIO_PIN_SET);
    delay_us(2);
    HAL_GPIO_WritePin(GPIOB, Y_STEP_PIN, GPIO_PIN_RESET);
}

/**
  * @brief  Ejecuta un solo paso en el eje Z
  * @retval None
  */
void Z_stepOnce(void) {
    HAL_GPIO_WritePin(GPIOA, Z_STEP_PIN, GPIO_PIN_SET);
    delay_us(2);
    HAL_GPIO_WritePin(GPIOA, Z_STEP_PIN, GPIO_PIN_RESET);
}

/**
  * @brief  Mueve el eje X una cantidad específica de pasos
  * @param  steps: Número de pasos a mover
  * @param  dir: Dirección del movimiento (true = positivo, false = negativo)
  * @retval None
  */
void X_move(int32_t steps, bool dir) {
    // Configura dirección
    HAL_GPIO_WritePin(GPIOB, X_DIR_PIN, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);

    // Ejecuta los pasos
    for (int32_t i = 0; i < steps; i++) {
        X_stepOnce();
        delay_us(STEP_DELAY_US);
    }
}

/**
  * @brief  Mueve el eje Y una cantidad específica de pasos
  * @param  steps: Número de pasos a mover
  * @param  dir: Dirección del movimiento (true = positivo, false = negativo)
  * @retval None
  */
void Y_move(int32_t steps, bool dir) {
    // Configura dirección
    HAL_GPIO_WritePin(GPIOB, Y_DIR_PIN, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
    // Ejecuta los pasos
    for (int32_t i = 0; i < steps; i++) {
        Y_stepOnce();
        delay_us(STEP_DELAY_US);
    }
}

/**
  * @brief  Mueve el eje Z una cantidad específica de pasos
  * @param  steps: Número de pasos a mover
  * @param  dir: Dirección del movimiento (true = positivo, false = negativo)
  * @retval None
  */
void Z_move(int32_t steps, bool dir) {
    // Configura dirección
    HAL_GPIO_WritePin(GPIOA, Z_DIR_PIN, dir ? GPIO_PIN_RESET : GPIO_PIN_SET);
    // Ejecuta los pasos
    for (int32_t i = 0; i < steps; i++) {
        Z_stepOnce();
        delay_us(STEP_DELAY_US);
    }
}

// ===========================================================================================
// FUNCIONES DE CONTROL DE MOTORES
// ===========================================================================================

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

// ===========================================================================================
// FUNCIONES DE MOVIMIENTO COORDINADO
// ===========================================================================================

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
    deltaX = (deltaX < 0) ? -deltaX : deltaX;
    deltaY = (deltaY < 0) ? -deltaY : deltaY;
    deltaZ = (deltaZ < 0) ? -deltaZ : deltaZ;
    
    // Calcular delay basado en feed rate
    uint32_t step_delay = calculateStepDelay(effective_feedrate, total_distance);
    
    // Mostrar información del movimiento con validación de valores
    // float display_x = !isnan(x) ? x : (currentX / (float)STEPS_PER_MM_X);
    // float display_y = !isnan(y) ? y : (currentY / (float)STEPS_PER_MM_Y);
    // float display_z = !isnan(z) ? z : (currentZ / (float)STEPS_PER_MM_Z);
    
    // Convertir floats a enteros para evitar problemas de printf con floats
    // int x_int = (int)display_x;
    // int x_dec = (int)((display_x - x_int) * 100);
    // int y_int = (int)display_y;
    // int y_dec = (int)((display_y - y_int) * 100);
    // int z_int = (int)display_z;
    // int z_dec = (int)((display_z - z_int) * 100);
    // int f_int = (int)effective_feedrate;
    // int f_dec = (int)((effective_feedrate - f_int) * 10);
    // int d_int = (int)total_distance;
    // int d_dec = (int)((total_distance - d_int) * 100);

    // snprintf(outputBuffer, OUTPUT_BUFFER_SIZE, "%s: X=%d.%02d Y=%d.%02d Z=%d.%02d F=%d.%d D=%d.%02dmm T=%lduS\r\n", 
    //        isRapid ? "G0 RAPID" : "G1 LINEAR",
    //        x_int, abs(x_dec),
    //        y_int, abs(y_dec), 
    //        z_int, abs(z_dec),
    //        f_int, abs(f_dec), 
    //        d_int, abs(d_dec), 
    //        (unsigned long)step_delay);
    // sendUSBText(outputBuffer);
    // memset(outputBuffer, 0, OUTPUT_BUFFER_SIZE);
    
    // Algoritmo de interpolación lineal 3D (Bresenham modificado)
    int32_t maxSteps = deltaX;
    if (deltaY > maxSteps) maxSteps = deltaY;
    if (deltaZ > maxSteps) maxSteps = deltaZ;
    
    if (maxSteps == 0) return; // No hay movimiento
    
    // Variables para el algoritmo de Bresenham 3D
    int32_t errorX = maxSteps / 2;
    int32_t errorY = maxSteps / 2;
    int32_t errorZ = maxSteps / 2;
    
    
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
    
    
    // Actualizar posiciones actuales
    currentX = targetX;
    currentY = targetY;
    currentZ = targetZ;
}

// ===========================================================================================
// FUNCIONES DE MOVIMIENTO DE ARCOS
// ===========================================================================================

/**
  * @brief  Movimiento de arco con radio específico
  * @param  x_end, y_end: Coordenadas finales del arco en mm
  * @param  r: Radio del arco en mm
  * @param  clockwise: Dirección del arco (1 = horario, 0 = antihorario)
  * @retval None
  */
void arc_move_r(float x_end, float y_end, float r, bool clockwise) {
    float x0 = currentX;
    float y0 = currentY;
    float x1 = x_end * STEPS_PER_MM_X;
    float y1 = y_end * STEPS_PER_MM_Y;
    r = r * STEPS_PER_MM_X; // Convertir radio a pasos

    float dx = x1 - x0;
    float dy = y1 - y0;
    float d = sqrt(dx * dx + dy * dy);

    if (d > 2 * fabs(r)) {
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
        cx = mx - nx * h;
        cy = my - ny * h;
    } else {
        cx = mx + nx * h;
        cy = my + ny * h;
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

// ===========================================================================================
// FUNCIONES DE UTILIDAD
// ===========================================================================================

/**
  * @brief  Obtener posición actual en milímetros
  * @param  x, y, z: Punteros para almacenar las coordenadas actuales
  * @retval None
  */
void getCurrentPositionMM(float *x, float *y, float *z) {
    if (x) *x = currentX / (float)STEPS_PER_MM_X;
    if (y) *y = currentY / (float)STEPS_PER_MM_Y;
    if (z) *z = currentZ / (float)STEPS_PER_MM_Z;
}

/**
  * @brief  Establecer posición actual en pasos
  * @param  x_steps, y_steps, z_steps: Nueva posición en pasos
  * @retval None
  */
void setCurrentPosition(int32_t x_steps, int32_t y_steps, int32_t z_steps) {
    currentX = x_steps;
    currentY = y_steps;
    currentZ = z_steps;
}

/**
  * @brief  Establecer posición actual en milímetros
  * @param  x_mm, y_mm, z_mm: Nueva posición en milímetros
  * @retval None
  */
void setCurrentPositionMM(float x_mm, float y_mm, float z_mm) {
    currentX = (int32_t)(x_mm * STEPS_PER_MM_X);
    currentY = (int32_t)(y_mm * STEPS_PER_MM_Y);
    currentZ = (int32_t)(z_mm * STEPS_PER_MM_Z);
}

//Soporte revisar gcode

// typedef struct {
//     double x, y, z, f;
//     int tiene_x, tiene_y, tiene_z, tiene_f;
// } Punto;


// typedef struct {
//     double x, y;
// } Vec2;

// Vec2 resta(Vec2 a, Vec2 b) {
//     Vec2 r = { a.x - b.x, a.y - b.y };
//     return r;
// }

// Vec2 suma(Vec2 a, Vec2 b) {
//     Vec2 s = { a.x + b.x, a.y + b.y };
//     return s;
// }

// Vec2 escalar(Vec2 v, double f) {
//     Vec2 r = { v.x * f, v.y * f };
//     return r;
// }

// Vec2 normalizar(Vec2 v) {
//     double mag = hypot(v.x, v.y);
//     Vec2 n = { v.x / mag, v.y / mag };
//     return n;
// }

// double dot(Vec2 a, Vec2 b) {
//     return a.x * b.x + a.y * b.y;
// }

// double cross(Vec2 a, Vec2 b) {
//     return a.x * b.y - a.y * b.x;
// }

// double grados(double rad) {
//     return rad * 180.0 / PI;
// }

// Punto parsear_linea_g1(const char *linea, double lastX, double lastY, double lastZ, double lastF) {
//     Punto p = {lastX, lastY, lastZ, lastF, 0, 0, 0, 0};
//     char letra;
//     double valor;
//     const char *ptr = linea;

//     while (*ptr) {
//         if (sscanf(ptr, " %c%lf", &letra, &valor) == 2) {
//             switch (letra) {
//                 case 'X': p.x = valor; p.tiene_x = 1; break;
//                 case 'Y': p.y = valor; p.tiene_y = 1; break;
//                 case 'Z': p.z = valor; p.tiene_z = 1; break;
//                 case 'F': p.f = valor; p.tiene_f = 1; break;
//             }
//         }

//         // Saltar al siguiente bloque
//         while (*ptr && *ptr != ' ') ptr++;
//         while (*ptr == ' ') ptr++;
//     }

//     return p;
// }


// void suavizarLineaGcode(
//     const char *lineaAnterior,
//     const char *lineaActual,
//     char gcodeSuavizado[][MAX_LINE_LENGTH],
//     int *nuevaLinea
// ) {
//     // Parsear ambas líneas
//     static double lastX = 0, lastY = 0, lastZ = 0, lastF = 0;
//     Punto p0 = parsear_linea_g1(lineaAnterior, lastX, lastY, lastZ, lastF);
//     if (p0.tiene_x) lastX = p0.x;
//     if (p0.tiene_y) lastY = p0.y;
//     if (p0.tiene_z) lastZ = p0.z;
//     if (p0.tiene_f) lastF = p0.f;

//     p0.x = lastX; p0.y = lastY; p0.z = lastZ; p0.f = lastF;

//     Punto p1 = parsear_linea_g1(lineaActual, lastX, lastY, lastZ, lastF);
//     if (p1.tiene_x) lastX = p1.x;
//     if (p1.tiene_y) lastY = p1.y;
//     if (p1.tiene_z) lastZ = p1.z;
//     if (p1.tiene_f) lastF = p1.f;

//     p1.x = lastX; p1.y = lastY; p1.z = lastZ; p1.f = lastF;

//     // Vector entre p0 y p1
//     Vec2 v = resta((Vec2){p1.x, p1.y}, (Vec2){p0.x, p0.y});

//     // Si el movimiento es corto, o en línea recta, no suavizar
//     double distancia = hypot(v.x, v.y);
//     if (distancia < RADIO_SUAVIZADO) {
//         strcpy(gcodeSuavizado[(*nuevaLinea)++], lineaActual);
//         return;
//     }

//     // Si querés suavizar:
//     Vec2 vNorm = normalizar(v);
//     Vec2 p0a = suma((Vec2){p0.x, p0.y}, escalar(vNorm, RADIO_SUAVIZADO));
//     Vec2 p1b = resta((Vec2){p1.x, p1.y}, escalar(vNorm, RADIO_SUAVIZADO));

//     char linea[MAX_LINE_LENGTH];

//     // Línea hasta punto de entrada del arco
//     sprintf(linea, "G1 X%.3f Y%.3f", p0a.x, p0a.y);
//     if (p0.tiene_z) sprintf(linea + strlen(linea), " Z%.3f", p0.z);
//     if (p0.tiene_f) sprintf(linea + strlen(linea), " F%.3f", p0.f);
//     strcpy(gcodeSuavizado[(*nuevaLinea)++], linea);

//     // Arco de entrada
//     sprintf(linea, "G2 X%.3f Y%.3f R%.3f", p1b.x, p1b.y, RADIO_SUAVIZADO);
//     strcpy(gcodeSuavizado[(*nuevaLinea)++], linea);
// }

// void revisar_gcode(const char gcodeProgram[][MAX_LINE_LENGTH], int numLineas, char gcodeSuavizado[][MAX_LINE_LENGTH], int *numLineasSuavizado) {
//     int nuevaLinea = 0;

//     for (int i = 1; i < numLineas; ++i) {
//         // Procesar sólo pares consecutivos G0/G1
//         if ((strncmp(gcodeProgram[i - 1], "G0", 2) == 0 || strncmp(gcodeProgram[i - 1], "G1", 2) == 0) &&
//             (strncmp(gcodeProgram[i], "G0", 2) == 0 || strncmp(gcodeProgram[i], "G1", 2) == 0)) {
//             suavizarLineaGcode(gcodeProgram[i - 1], gcodeProgram[i], gcodeSuavizado, &nuevaLinea);
//         } else {
//             // Copiar línea tal cual si no es G0/G1 o es la primera
//             strcpy(gcodeSuavizado[nuevaLinea++], gcodeProgram[i]);
//         }
//     }

//     *numLineasSuavizado = nuevaLinea;
// }
