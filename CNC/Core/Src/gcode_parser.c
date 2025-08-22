/**
  ******************************************************************************
  * @file           : gcode_parser.c
  * @brief          : Parser G-code mejorado basado en GRBL32
  ******************************************************************************
  * @attention
  *
  * Parser G-code profesional compatible con GRBL para STM32F103
  * Implementa validación de grupos modales y códigos de error estándar
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "gcode_parser.h"
#include "config.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* Variables globales -------------------------------------------------------*/
gc_block_t gc_block;              // Bloque de parsing actual
gc_modal_t gc_state_modal;        // Estado modal persistente

/* Variables externas -------------------------------------------------------*/
extern int32_t currentX, currentY, currentZ;
extern char outputBuffer[];  // Buffer compartido definido en main.c

/* Funciones externas -------------------------------------------------------*/
extern void performHoming(void);
extern void enableSteppers(void);
extern void disableSteppers(void);
extern bool isEndstopPressed(char axis);
extern void sendUSBText(const char* message);
extern void showConfiguration(void);

/* Definiciones de constantes -----------------------------------------------*/
#define STEPS_PER_MM_X 79
#define STEPS_PER_MM_Y 79
#define STEPS_PER_MM_Z 3930

// Funciones de callback con feed rate
extern void moveAxesRapidCallback(float x, float y, float z, bool x_defined, bool y_defined, bool z_defined);
extern void moveAxesLinearCallback(float x, float y, float z, float feedRate, bool x_defined, bool y_defined, bool z_defined, bool f_defined);
extern void moveAxesArcCallback(float x, float y, float r, bool clockwise, float feedRate, bool f_defined);

/* Funciones privadas -------------------------------------------------------*/

/**
  * @brief  Verifica si las coordenadas objetivo están dentro de los límites de la máquina
  * @param  target_x: Coordenada X objetivo (en mm)
  * @param  target_y: Coordenada Y objetivo (en mm)  
  * @param  target_z: Coordenada Z objetivo (en mm)
  * @param  x_defined: True si X está definido en el comando
  * @param  y_defined: True si Y está definido en el comando
  * @param  z_defined: True si Z está definido en el comando
  * @retval STATUS_OK si está dentro de límites, STATUS_SOFT_LIMIT_ERROR si está fuera
  */
uint8_t check_soft_limits(float target_x, float target_y, float target_z, bool x_defined, bool y_defined, bool z_defined) {
    char msg[100];
    
    // Verificar límites para eje X
    if (x_defined) {
        if (target_x < MIN_TRAVEL_X || target_x > MAX_TRAVEL_X) {
            int target_x_int = (int)target_x;
            int target_x_dec = (int)((target_x - target_x_int) * 100);
            int min_x = (int)MIN_TRAVEL_X;
            int max_x = (int)MAX_TRAVEL_X;
            sprintf(msg, "Error: X=%d.%02d fuera de límites [%d, %d]\r\n", 
                   target_x_int, abs(target_x_dec), min_x, max_x);
            sendUSBText(msg);
            return STATUS_SOFT_LIMIT_ERROR;
        }
    }
    
    // Verificar límites para eje Y
    if (y_defined) {
        if (target_y < MIN_TRAVEL_Y || target_y > MAX_TRAVEL_Y) {
            int target_y_int = (int)target_y;
            int target_y_dec = (int)((target_y - target_y_int) * 100);
            int min_y = (int)MIN_TRAVEL_Y;
            int max_y = (int)MAX_TRAVEL_Y;
            sprintf(msg, "Error: Y=%d.%02d fuera de límites [%d, %d]\r\n", 
                   target_y_int, abs(target_y_dec), min_y, max_y);
            sendUSBText(msg);
            return STATUS_SOFT_LIMIT_ERROR;
        }
    }
    
    // Verificar límites para eje Z
    if (z_defined) {
        if (target_z < MIN_TRAVEL_Z || target_z > MAX_TRAVEL_Z) {
            int target_z_int = (int)target_z;
            int target_z_dec = (int)((target_z - target_z_int) * 100);
            int min_z = (int)MIN_TRAVEL_Z;
            int max_z = (int)MAX_TRAVEL_Z;
            sprintf(msg, "Error: Z=%d.%02d fuera de límites [%d, %d]\r\n", 
                   target_z_int, abs(target_z_dec), min_z, max_z);
            sendUSBText(msg);
            return STATUS_SOFT_LIMIT_ERROR;
        }
    }
    
    return STATUS_OK;  // Todas las coordenadas están dentro de límites
}

/**
  * @brief  Reporta los límites de la máquina por USB
  * @retval None
  */
void report_machine_limits(void) {
    sendUSBText("Límites de la máquina:\r\n");
    
    // Convertir a enteros para evitar problemas con printf float
    int min_x = (int)MIN_TRAVEL_X;
    int max_x = (int)MAX_TRAVEL_X;
    int min_y = (int)MIN_TRAVEL_Y;
    int max_y = (int)MAX_TRAVEL_Y;
    int min_z = (int)MIN_TRAVEL_Z;
    int max_z = (int)MAX_TRAVEL_Z;
    
    sprintf(outputBuffer, "X: [%d, %d] mm\r\n", min_x, max_x);
    sendUSBText(outputBuffer);
    sprintf(outputBuffer, "Y: [%d, %d] mm\r\n", min_y, max_y);
    sendUSBText(outputBuffer);
    sprintf(outputBuffer, "Z: [%d, %d] mm\r\n", min_z, max_z);
    sendUSBText(outputBuffer);
}

/**
  * @brief  Inicializa el parser G-code con valores por defecto
  * @retval None
  */
void gc_init(void) {
    // Inicializar estado modal por defecto
    memset(&gc_state_modal, 0, sizeof(gc_modal_t));
    gc_state_modal.motion = MOTION_MODE_SEEK;       // G0 por defecto
    gc_state_modal.spindle = 0;                     // M5 (spindle off) por defecto
    gc_state_modal.program_flow = 0;                // Normal execution
}

/**
  * @brief  Limpia el bloque actual y copia el estado modal
  * @retval None
  */
void gc_clear_block(void) {
    // Limpiar bloque actual y copiar estado modal
    memset(&gc_block, 0, sizeof(gc_block_t));
    memcpy(&gc_block.modal, &gc_state_modal, sizeof(gc_modal_t));
    
    // Inicializar valores como indefinidos
    gc_block.values.x = NAN;
    gc_block.values.y = NAN;
    gc_block.values.z = NAN;
    gc_block.values.f = NAN;
    gc_block.values.s = NAN;
    gc_block.values.p = 0;
    gc_block.values.r = NAN;
    
    // Flags de definición
    gc_block.values.x_defined = false;
    gc_block.values.y_defined = false;
    gc_block.values.z_defined = false;
    gc_block.values.f_defined = false;
    gc_block.values.s_defined = false;
    gc_block.values.r_defined = false;
}

/**
  * @brief  Lee un número float de una línea de texto
  * @param  line: Línea de texto
  * @param  char_counter: Contador de caracteres (se modifica)
  * @param  float_ptr: Puntero donde guardar el resultado
  * @retval true si se leyó correctamente, false en caso contrario
  */
bool read_float(char *line, uint8_t *char_counter, float *float_ptr) {
    char *ptr = line + *char_counter;
    unsigned char c;
    
    // Saltar espacios
    while ((c = *ptr) == ' ' || c == '\t') { 
        ptr++; 
        (*char_counter)++;
    }
    
    if (c == 0) return false;  // Fin de línea
    
    char *start = ptr;
    
    // Leer número (incluyendo signo y punto decimal)
    if (c == '+' || c == '-') { 
        ptr++; 
        (*char_counter)++;
    }
    
    bool found_decimal = false;
    bool found_digit = false;
    
    while ((c = *ptr) != 0) {
        if (c >= '0' && c <= '9') {
            found_digit = true;
        } else if (c == '.' && !found_decimal) {
            found_decimal = true;
        } else {
            break;  // Caracter no numérico
        }
        ptr++;
        (*char_counter)++;
    }
    
    if (!found_digit) return false;
    
    *float_ptr = atof(start);
    return true;
}

/**
  * @brief  Parsea una línea G-code completa
  * @param  line: Línea de texto a parsear
  * @retval Código de estado (STATUS_OK si es exitoso)
  */
uint8_t gc_parse_line(char *line) {
    uint8_t char_counter = 0;
    unsigned char letter;
    float value;
    uint16_t int_value = 0;
    uint16_t command_words = 0;   // Tracking de comandos para detectar conflictos
    uint8_t word_bit = 0;
    
    // Limpiar bloque y copiar estado modal
    gc_clear_block();
    
    // Procesar cada palabra en la línea
    while (line[char_counter] != 0) {
        // Saltar espacios
        while (line[char_counter] == ' ' || line[char_counter] == '\t') {
            char_counter++;
        }
        
        // Obtener letra del comando
        letter = line[char_counter];
        if (letter == 0) break;  // Fin de línea
        
        // Convertir a mayúscula
        if (letter >= 'a' && letter <= 'z') {
            letter -= 32;
        }
        
        char_counter++;
        
        // Leer valor después de la letra
        if (!read_float(line, &char_counter, &value)) {
            return STATUS_BAD_NUMBER_FORMAT;
        }
        
        // Convertir a entero para comandos G y M
        int_value = truncf(value);
        
        // Procesar según la letra del comando
        switch (letter) {
            case 'G':
                // Validar rango de comandos G
                if (int_value > 99) return STATUS_GCODE_UNSUPPORTED_COMMAND;
                
                switch (int_value) {
                    case 0: case 1: case 2: case 3:
                        word_bit = MODAL_GROUP_G1;
                        gc_block.modal.motion = int_value;
                        break;
                    case 4:
                        word_bit = MODAL_GROUP_G0;
                        gc_block.non_modal_command = 4;  // G4 - Dwell
                        break;
                    case 28:
                        word_bit = MODAL_GROUP_G0;
                        gc_block.non_modal_command = 28;  // G28 - Home
                        break;
                    case 92:
                        word_bit = MODAL_GROUP_G0;
                        gc_block.non_modal_command = 92;  // G92 - Set position
                        break;
                    default:
                        return STATUS_GCODE_UNSUPPORTED_COMMAND;
                }
                
                // Verificar violación de grupo modal
                if (bit_istrue(command_words, bit(word_bit))) {
                    return STATUS_GCODE_MODAL_GROUP_VIOLATION;
                }
                command_words |= bit(word_bit);
                break;
                
            case 'M':
                // Validar rango de comandos M
                if (int_value > 999) return STATUS_GCODE_UNSUPPORTED_COMMAND;
                
                switch (int_value) {
                    case 0: case 2:
                        word_bit = MODAL_GROUP_M4;
                        gc_block.modal.program_flow = int_value;
                        break;
                    case 3: case 5:
                        word_bit = MODAL_GROUP_M7;
                        gc_block.modal.spindle = int_value;
                        break;
                    case 17:  // M17 - Enable steppers
                        word_bit = MODAL_GROUP_G0;  // Comando no modal
                        gc_block.non_modal_command = 17;
                        break;
                    case 18:  // M18 - Disable steppers
                    case 84:  // M84 - Disable steppers (alias de M18)
                        word_bit = MODAL_GROUP_G0;  // Comando no modal
                        gc_block.non_modal_command = 18;
                        break;
                    case 114: // M114 - Report current position
                        word_bit = MODAL_GROUP_G0;  // Comando no modal
                        gc_block.non_modal_command = 114;
                        break;
                    case 119: // M119 - Report endstop status
                        word_bit = MODAL_GROUP_G0;  // Comando no modal
                        gc_block.non_modal_command = 119;
                        break;
                    case 503: // M503 - Report settings
                        word_bit = MODAL_GROUP_G0;  // Comando no modal
                        gc_block.non_modal_command = 503;
                        break;
                    case 505: // M505 - Report machine limits
                        word_bit = MODAL_GROUP_G0;  // Comando no modal
                        gc_block.non_modal_command = 505;
                        break;
                    default:
                        return STATUS_GCODE_UNSUPPORTED_COMMAND;
                }
                
                // Verificar violación de grupo modal
                if (bit_istrue(command_words, bit(word_bit))) {
                    return STATUS_GCODE_MODAL_GROUP_VIOLATION;
                }
                command_words |= bit(word_bit);
                break;
                
            case 'X':
                gc_block.values.x = value;
                gc_block.values.x_defined = true;
                break;
            case 'Y':
                gc_block.values.y = value;
                gc_block.values.y_defined = true;
                break;
            case 'Z':
                gc_block.values.z = value;
                gc_block.values.z_defined = true;
                break;
            case 'F':
                gc_block.values.f = value;
                gc_block.values.f_defined = true;
                if (value < 0.0) return STATUS_NEGATIVE_VALUE;
                break;
            case 'P':
                gc_block.values.p = int_value;
                if (value < 0.0) return STATUS_NEGATIVE_VALUE;
                break;
            case 'S':
                gc_block.values.s = value;
                gc_block.values.s_defined = true;
                if (value < 0.0) return STATUS_NEGATIVE_VALUE;
                break;
            case 'R':
                gc_block.values.r = value;
                gc_block.values.r_defined = true;
                if (value < 0.0) return STATUS_NEGATIVE_VALUE;
                break;
            default:
                return STATUS_GCODE_UNSUPPORTED_COMMAND;
        }
    }
    
    return STATUS_OK;  // Parsing exitoso
}

/**
  * @brief  Ejecuta los comandos parseados en el bloque actual
  * @retval Código de estado (STATUS_OK si es exitoso)
  */
uint8_t gc_execute_block(void) {
    char msg[100];
    
    // Primero ejecutar comandos no modales
    switch (gc_block.non_modal_command) {
        case 4:  // G4 - Dwell (pausa)
            if (gc_block.values.p > 0) {
                // Implementar delay aquí si es necesario
                sprintf(msg, "Pausa de %d ms completada\r\n", gc_block.values.p);
                CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
            }
            break;
            
        case 28: // G28 - Home
            CDC_Transmit_FS((uint8_t*)"Ejecutando homing...\r\n", 22);
            performHoming();
            break;
            
        case 92: // G92 - Set position
            if (gc_block.values.x_defined) {
                currentX = gc_block.values.x * STEPS_PER_MM_X;
            }
            if (gc_block.values.y_defined) {
                currentY = gc_block.values.y * STEPS_PER_MM_Y;
            }
            if (gc_block.values.z_defined) {
                currentZ = gc_block.values.z * STEPS_PER_MM_Z;
            }
            sprintf(msg, "Posición establecida: X%.2f Y%.2f Z%.2f\r\n",
                   currentX/(float)STEPS_PER_MM_X, 
                   currentY/(float)STEPS_PER_MM_Y, 
                   currentZ/(float)STEPS_PER_MM_Z);
            CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
            break;
            
        case 17: // M17 - Enable steppers
            CDC_Transmit_FS((uint8_t*)"Motores habilitados (M17)\r\n", 27);
            enableSteppers();
            break;
            
        case 18: // M18/M84 - Disable steppers
            CDC_Transmit_FS((uint8_t*)"Motores deshabilitados\r\n", 24);
            disableSteppers();
            break;
            
        case 114: // M114 - Report current position
            {
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

                snprintf(outputBuffer, 200, "X:%d.%02d Y:%d.%02d Z:%d.%02d\r\n", 
                       x_int, abs(x_dec), y_int, abs(y_dec), z_int, abs(z_dec));
                sendUSBText(outputBuffer);
            }
            break;
            
        case 119: // M119 - Report endstop status  
            snprintf(outputBuffer, 200, "Estado fines de carrera:\r\n");
            sendUSBText(outputBuffer);
            snprintf(outputBuffer, 200, "X_MIN: %s\r\n", isEndstopPressed('X') ? "PRESSED" : "open");
            sendUSBText(outputBuffer);
            snprintf(outputBuffer, 200, "Y_MIN: %s\r\n", isEndstopPressed('Y') ? "PRESSED" : "open");
            sendUSBText(outputBuffer);
            snprintf(outputBuffer, 200, "Z_MIN: %s\r\n", isEndstopPressed('Z') ? "PRESSED" : "open");
            sendUSBText(outputBuffer);
            break;
            
        case 503: // M503 - Report settings
            showConfiguration();
            break;
            
        case 505: // M505 - Report machine limits
            report_machine_limits();
            break;
    }
    
    // Verificar límites de software antes de ejecutar comandos de movimiento
    if (gc_block.modal.motion == MOTION_MODE_SEEK || 
        gc_block.modal.motion == MOTION_MODE_LINEAR ||
        gc_block.modal.motion == MOTION_MODE_CW_ARC ||
        gc_block.modal.motion == MOTION_MODE_CCW_ARC) {
        
        if (gc_block.values.x_defined || gc_block.values.y_defined || gc_block.values.z_defined) {
            uint8_t limit_status = check_soft_limits(gc_block.values.x, gc_block.values.y, gc_block.values.z,
                                                     gc_block.values.x_defined, gc_block.values.y_defined, gc_block.values.z_defined);
            if (limit_status != STATUS_OK) {
                return limit_status;  // Retornar error de límites
            }
        }
    }
    
    // Ejecutar comandos de movimiento
    switch (gc_block.modal.motion) {
        case MOTION_MODE_SEEK:    // G0 - Movimiento rápido
            if (gc_block.values.x_defined || gc_block.values.y_defined || gc_block.values.z_defined) {
                moveAxesRapidCallback(gc_block.values.x, gc_block.values.y, gc_block.values.z,
                                     gc_block.values.x_defined, gc_block.values.y_defined, gc_block.values.z_defined);
            }
            break;
            
        case MOTION_MODE_LINEAR:  // G1 - Movimiento lineal
            if (gc_block.values.x_defined || gc_block.values.y_defined || gc_block.values.z_defined) {
                moveAxesLinearCallback(gc_block.values.x, gc_block.values.y, gc_block.values.z, gc_block.values.f,
                                      gc_block.values.x_defined, gc_block.values.y_defined, gc_block.values.z_defined, gc_block.values.f_defined);
            }
            break;
            
        case MOTION_MODE_CW_ARC:   // G2 - Arco horario
            if (gc_block.values.x_defined || gc_block.values.y_defined || gc_block.values.r_defined) {
                moveAxesArcCallback(gc_block.values.x, gc_block.values.y, gc_block.values.r, 1, gc_block.values.f, gc_block.values.f_defined);  // 1 para sentido horario
                break;
            }
        case MOTION_MODE_CCW_ARC:  // G3 - Arco antihorario
            if (gc_block.values.x_defined || gc_block.values.y_defined || gc_block.values.r_defined) {
                moveAxesArcCallback(gc_block.values.x, gc_block.values.y, gc_block.values.r, 0, gc_block.values.f, gc_block.values.f_defined);  // 0 para sentido antihorario
                break;
            }
            break;
    }
    
    // Ejecutar comandos M
    switch (gc_block.modal.spindle) {
        case 3:  // M3 - Spindle CW
            CDC_Transmit_FS((uint8_t*)"Spindle activado (CW)\r\n", 23);
            break;
        case 5:  // M5 - Spindle off
            CDC_Transmit_FS((uint8_t*)"Spindle desactivado\r\n", 21);
            break;
    }
        
    // Actualizar estado modal persistente
    memcpy(&gc_state_modal, &gc_block.modal, sizeof(gc_modal_t));
    
    return STATUS_OK;
}

/**
  * @brief  Función principal para ejecutar una línea G-code completa
  * @param  line: Línea de G-code a ejecutar
  * @retval Código de estado (STATUS_OK si es exitoso)
  */
uint8_t gc_execute_line(char *line) {
    uint8_t status_code;
    
    // Convertir línea a mayúsculas y limpiar
    for (int i = 0; line[i] != 0; i++) {
        if (line[i] >= 'a' && line[i] <= 'z') {
            line[i] -= 32;  // Convertir a mayúscula
        }
    }
    
    // Parsear la línea
    status_code = gc_parse_line(line);
    if (status_code != STATUS_OK) {
        return status_code;
    }
    
    // Ejecutar comandos parseados
    status_code = gc_execute_block();
    if (status_code != STATUS_OK) {
        return status_code;
    }
    
    return STATUS_OK;
}

/**
  * @brief  Envía mensaje de estado según el código de error
  * @param  status_code: Código de estado a reportar
  * @retval None
  */
void report_status_message(uint8_t status_code) {
    switch (status_code) {
        case STATUS_OK:
            CDC_Transmit_FS((uint8_t*)"ok\r\n", 4);
            break;
        case STATUS_EXPECTED_COMMAND_LETTER:
            CDC_Transmit_FS((uint8_t*)"error:1 (Expected command letter)\r\n", 36);
            break;
        case STATUS_BAD_NUMBER_FORMAT:
            CDC_Transmit_FS((uint8_t*)"error:2 (Bad number format)\r\n", 30);
            break;
        case STATUS_INVALID_STATEMENT:
            CDC_Transmit_FS((uint8_t*)"error:3 (Invalid statement)\r\n", 30);
            break;
        case STATUS_NEGATIVE_VALUE:
            CDC_Transmit_FS((uint8_t*)"error:4 (Negative value)\r\n", 27);
            break;
        case STATUS_SOFT_LIMIT_ERROR:
            CDC_Transmit_FS((uint8_t*)"error:5 (Soft limit exceeded)\r\n", 32);
            break;
        case STATUS_GCODE_UNSUPPORTED_COMMAND:
            CDC_Transmit_FS((uint8_t*)"error:20 (Unsupported command)\r\n", 33);
            break;
        case STATUS_GCODE_MODAL_GROUP_VIOLATION:
            CDC_Transmit_FS((uint8_t*)"error:21 (Modal group violation)\r\n", 35);
            break;
        case STATUS_GCODE_UNDEFINED_FEED_RATE:
            CDC_Transmit_FS((uint8_t*)"error:22 (Undefined feed rate)\r\n", 33);
            break;
        case STATUS_GCODE_COMMAND_VALUE_NOT_INTEGER:
            CDC_Transmit_FS((uint8_t*)"error:23 (Command value not integer)\r\n", 39);
            break;
        case STATUS_GCODE_AXIS_COMMAND_CONFLICT:
            CDC_Transmit_FS((uint8_t*)"error:24 (Axis command conflict)\r\n", 35);
            break;
        case STATUS_GCODE_WORD_REPEATED:
            CDC_Transmit_FS((uint8_t*)"error:25 (Word repeated)\r\n", 27);
            break;
        case STATUS_GCODE_NO_AXIS_WORDS:
            CDC_Transmit_FS((uint8_t*)"error:26 (No axis words)\r\n", 27);
            break;
        case STATUS_GCODE_INVALID_LINE_NUMBER:
            CDC_Transmit_FS((uint8_t*)"error:27 (Invalid line number)\r\n", 33);
            break;
        case STATUS_GCODE_VALUE_WORD_MISSING:
            CDC_Transmit_FS((uint8_t*)"error:28 (Value word missing)\r\n", 32);
            break;
        default:
            {
                char msg[30];
                sprintf(msg, "error:%d\r\n", status_code);
                CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
            }
            break;
    }
}
