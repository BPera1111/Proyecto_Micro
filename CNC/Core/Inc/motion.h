/*
  motion.h - Módulo de control de movimiento para CNC Controller
  
  Este archivo contiene todas las funciones relacionadas con el movimiento
  de los motores paso a paso y control de ejes.
*/

#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "config.h"
#include "main.h"

// ===========================================================================================
// DECLARACIONES EXTERNAS PARA MAIN.C
// ===========================================================================================

// Buffer global para comunicación USB (definido en main.c)
extern char outputBuffer[];

// Función de envío USB (definida en main.c)
extern void sendUSBText(const char* message);

// ===========================================================================================
// VARIABLES GLOBALES DE POSICIÓN
// ===========================================================================================

// Variables de posición actual en pasos
extern int32_t currentX, currentY, currentZ;

// Variables de velocidad
extern float currentFeedRate;
extern float rapidRate;
extern float maxFeedRate;

// ===========================================================================================
// FUNCIONES DE MOVIMIENTO BÁSICO POR PASOS
// ===========================================================================================

// Funciones para ejecutar un solo paso en cada eje
void X_stepOnce(void);
void Y_stepOnce(void);
void Z_stepOnce(void);

// Funciones para mover una cantidad específica de pasos
void X_move(int32_t steps, bool dir);
void Y_move(int32_t steps, bool dir);
void Z_move(int32_t steps, bool dir);

// ===========================================================================================
// FUNCIONES DE CONTROL DE MOTORES
// ===========================================================================================

// Habilitar/deshabilitar motores paso a paso
void enableSteppers(void);
void disableSteppers(void);

// ===========================================================================================
// FUNCIONES DE MOVIMIENTO COORDINADO
// ===========================================================================================

// Movimiento coordinado de múltiples ejes con control de velocidad
void moveAxesWithFeedRate(float x, float y, float z, float feedRate, bool isRapid);

// ===========================================================================================
// FUNCIONES DE MOVIMIENTO DE ARCOS
// ===========================================================================================

// Movimiento de arco con radio específico
void arc_move_r(float x_end, float y_end, float r, bool clockwise);

// ===========================================================================================
// FUNCIONES AUXILIARES DE VELOCIDAD
// ===========================================================================================

// Calcular delay entre pasos según la velocidad de alimentación
uint32_t calculateStepDelay(float feedRate, float distance_mm);

// Función de delay de microsegundos
void delay_us(uint32_t us);

// ===========================================================================================
// FUNCIONES DE UTILIDAD
// ===========================================================================================

// Obtener posición actual en milímetros
void getCurrentPositionMM(float *x, float *y, float *z);

// Establecer posición actual (para homing y G92)
void setCurrentPosition(int32_t x_steps, int32_t y_steps, int32_t z_steps);
void setCurrentPositionMM(float x_mm, float y_mm, float z_mm);

// Revisar G-code 
void revisar_gcode(const char gcodeProgram[][MAX_LINE_LENGTH], int numLineas, char gcodeSuavizado[][MAX_LINE_LENGTH], int *numLineasSuavizado);

#endif // MOTION_H
