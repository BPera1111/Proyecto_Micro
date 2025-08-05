# Ejemplo de uso del Planner Lookahead

Este documento explica cómo usar el nuevo módulo de planner lookahead implementado.

## Características del Planner

- **Buffer de 8 líneas**: Planifica hasta 8 comandos G-code por adelantado
- **Optimización de velocidades**: Calcula velocidades óptimas en las juntas entre movimientos
- **Soporte completo**: G0 (rápido), G1 (lineal), G2 (arco CW), G3 (arco CCW)
- **Configuración de arcos**: Soporta modo R (radio) para G2/G3
- **Suavizado de trayectoria**: Reduce vibraciones y mejora la precisión

## Comandos de Control del Planner

### Comandos Básicos
- `PLANNER_STATUS` - Muestra el estado actual del planner
- `PLANNER_ENABLE` - Habilita el planner lookahead (por defecto)
- `PLANNER_DISABLE` - Deshabilita el planner (modo compatibilidad)
- `PLANNER_SYNC` - Espera a que el buffer esté vacío
- `PLANNER_RESET` - Reinicia completamente el planner

### Información de Estado
```
PLANNER_STATUS
```
Muestra:
- Estado (habilitado/deshabilitado)
- Uso del buffer (bloques ocupados)
- Posición actual del planner
- Configuración de parámetros

## Ejemplo de Programa G-code Optimizado

```gcode
PLANNER_ENABLE
G28                    ; Homing
G0 X0 Y0 Z5           ; Posición inicial rápida
G1 X10 Y0 F100        ; Línea 1
G1 X10 Y10 F100       ; Línea 2 - El planner optimizará la junta
G2 X20 Y10 R5 F80     ; Arco con radio 5mm
G1 X30 Y20 F100       ; Línea final
G0 Z10                ; Levantar Z
PLANNER_SYNC          ; Esperar terminación
```

## Configuración Avanzada

### Parámetros del Planner (config.h)
```c
#define PLANNER_BUFFER_SIZE 8           // Tamaño del buffer
#define JUNCTION_DEVIATION 0.1f         // Desviación en juntas (mm)
#define ACCELERATION 1000.0f            // Aceleración (mm/min²)
#define MAX_VELOCITY_CHANGE 500.0f      // Cambio máx de velocidad
```

### Ventajas del Planner
1. **Movimientos suaves**: Elimina paradas bruscas entre segmentos
2. **Mayor velocidad**: Optimiza velocidades de paso entre movimientos
3. **Mejor acabado**: Reduce vibraciones y marcas en la superficie
4. **Eficiencia**: Procesa comandos de forma anticipada

### Modo Compatibilidad
Si necesitas el comportamiento anterior (sin planner):
```gcode
PLANNER_DISABLE
G1 X10 Y10 F100    ; Movimiento directo sin optimización
PLANNER_ENABLE     ; Volver a habilitar
```

## Monitoreo del Planner

### Durante la Ejecución
El planner funciona automáticamente en el loop principal, procesando hasta 2 bloques por ciclo para mantener fluidez.

### Diagnóstico
- `PLANNER_STATUS` muestra estadísticas en tiempo real
- Buffer lleno = comandos se rechazan temporalmente
- Buffer vacío = sistema esperando más comandos

## Integración con Programas Existentes

El planner es totalmente compatible con el sistema de programas existente:

```gcode
PROGRAM_START
G28
G0 X0 Y0 Z2
G1 X10 Y0 F100
G1 X10 Y10 F100
G1 X0 Y10 F100
G1 X0 Y0 F100
PROGRAM_STOP
PROGRAM_RUN
```

Los comandos dentro del programa se procesan automáticamente con el planner habilitado.
