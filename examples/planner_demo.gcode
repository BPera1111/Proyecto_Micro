; Ejemplo de programa G-code optimizado con Planner Lookahead
; Este programa demuestra las capacidades del planner con diferentes tipos de movimiento

; Verificar estado inicial del planner
PLANNER_STATUS

; Asegurar que el planner esté habilitado
PLANNER_ENABLE

; Secuencia de homing
G28

; Posición inicial
G0 X0 Y0 Z5

; === EJEMPLO 1: Cuadrado con esquinas suavizadas ===
; El planner optimizará las velocidades en las esquinas
G1 X20 Y0 F200
G1 X20 Y20 F200
G1 X0 Y20 F200
G1 X0 Y0 F200

; === EJEMPLO 2: Trayectoria con arcos (G2/G3) ===
; Movimiento a posición inicial para arcos
G0 X30 Y0 Z2

; Arco horario (G2) con radio
G2 X40 Y10 R10 F150

; Línea recta
G1 X50 Y10 F150

; Arco antihorario (G3) con radio
G3 X60 Y0 R10 F150

; === EJEMPLO 3: Patrón zigzag con optimización ===
G0 X0 Y25 Z2
G1 X10 Y25 F180
G1 X10 Y30 F180
G1 X20 Y30 F180
G1 X20 Y35 F180
G1 X30 Y35 F180
G1 X30 Y40 F180

; === EJEMPLO 4: Círculo completo con segmentos ===
G0 X40 Y40 Z2

; Cuarto de círculo usando G2 (múltiples arcos pequeños)
G2 X45 Y45 R5 F100
G2 X40 Y50 R5 F100
G2 X35 Y45 R5 F100
G2 X40 Y40 R5 F100

; === EJEMPLO 5: Cambios de velocidad graduales ===
G0 X0 Y50 Z2
G1 X10 Y50 F50    ; Velocidad baja
G1 X20 Y50 F100   ; Velocidad media
G1 X30 Y50 F200   ; Velocidad alta
G1 X40 Y50 F100   ; Vuelta a velocidad media
G1 X50 Y50 F50    ; Vuelta a velocidad baja

; Posición final segura
G0 Z10
G0 X0 Y0

; Sincronizar planner (esperar que termine todo)
PLANNER_SYNC

; Mostrar estadísticas finales
PLANNER_STATUS
