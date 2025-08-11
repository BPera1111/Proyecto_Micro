# Sistema de Límites de Software - CNC Controller

## Resumen de Implementación

Se ha implementado un sistema completo de verificación de límites de software para proteger la máquina CNC de movimientos fuera de rango.

## Cambios Realizados

### 1. **Configuración de Límites (config.h)**
- Agregadas definiciones de límites de la máquina:
  - X: 0 - 160 mm
  - Y: 0 - 160 mm  
  - Z: 0 - 160 mm
- Margen de seguridad configurado en 2mm

### 2. **Parser G-code (gcode_parser.h y gcode_parser.c)**
- Nuevo código de error: `STATUS_SOFT_LIMIT_ERROR` (error:5)
- Función `check_soft_limits()`: Verifica coordenadas antes de movimiento
- Función `report_machine_limits()`: Muestra límites por USB
- Verificación automática antes de ejecutar G0, G1, G2, G3

### 3. **Comando M505 (main.c)**
- Nuevo comando M505 para mostrar límites de la máquina
- Documentación agregada en ayuda del sistema

### 4. **Archivo de Prueba**
- `examples/test_limites.gcode`: Archivo para probar el sistema de límites
- Incluye casos válidos e inválidos (comentados)

## Cómo Funciona

### Verificación Durante la Carga
**NUEVO**: Antes de cargar cualquier línea al programa, el sistema:
1. Parsea la línea G-code para extraer coordenadas
2. Verifica si las coordenadas están dentro de límites
3. Si una línea viola límites, **detiene inmediatamente** la carga del programa
4. Cancela todo el programa y muestra mensaje de error específico

### Verificación Durante la Ejecución
Durante la ejecución de comandos individuales, el sistema:
1. Verifica si las coordenadas están dentro de límites
2. Si están fuera de rango, devuelve error y cancela el movimiento
3. Muestra mensaje específico indicando qué eje está fuera de límites

### Comandos de Usuario
- **M505**: Muestra los límites actuales de la máquina
- **M114**: Muestra posición actual (ya existía)

### Mensajes de Error

**Durante la carga:**
```
error: línea 3 viola límites - 
Error: X=200.00 fuera de límites [0.0, 160.0]
error:5 (Soft limit exceeded)
Carga de programa cancelada
```

**Durante la ejecución:**
```
Error: X=200.00 fuera de límites [0.0, 160.0]
error:5 (Soft limit exceeded)
```

## Configuración de Límites

Para cambiar los límites, editar en `config.h`:
```c
#define MAX_TRAVEL_X 160.0f    // Nuevo límite X
#define MAX_TRAVEL_Y 160.0f    // Nuevo límite Y  
#define MAX_TRAVEL_Z 160.0f    // Nuevo límite Z
```

## Archivos de Prueba

### Comandos Válidos (deben funcionar):
- `examples/test_limites.gcode`: Pruebas dentro de límites
```gcode
G0 X0 Y0 Z0       ; Origen
G0 X160 Y160 Z160 ; Máximo permitido
G1 X80 Y80 F200   ; Centro
```

### Comandos Inválidos (deben fallar durante la carga):
- `examples/test_limite_carga.gcode`: Falla en línea 3 (X=200)
- `examples/test_multiples_errores.gcode`: Falla en línea 3 (X=-10)

```gcode
G0 X200 Y0 Z0     ; X fuera de rango - DETIENE CARGA
G0 X0 Y-10 Z0     ; Y negativo - DETIENE CARGA
G0 X0 Y0 Z200     ; Z fuera de rango - DETIENE CARGA
```

## Beneficios

1. **Protección Proactiva**: Detecta errores **antes** de cargar programas inválidos
2. **Prevención de Daños**: Evita que programas defectuosos lleguen a ejecutarse
3. **Feedback Inmediato**: Indica exactamente en qué línea y qué coordenada está mal
4. **Ahorro de Tiempo**: No necesitas esperar a la ejecución para detectar errores
5. **Compatible con GRBL**: Usa códigos de error estándar
6. **Configurable**: Fácil de ajustar para diferentes máquinas
7. **Doble Protección**: Verificación en carga + verificación en ejecución

## Recomendaciones

1. **Probar archivos de prueba**: Usar `test_limite_carga.gcode` y `test_multiples_errores.gcode`
2. **Verificar límites**: Asegurar que coincidan con las dimensiones físicas
3. **Doble verificación**: El sistema protege tanto en carga como en ejecución
4. **Flujo de trabajo**: Cargar programa → Si pasa validación → Ejecutar
5. **Implementar límites físicos**: Considerar switches como respaldo futuro
