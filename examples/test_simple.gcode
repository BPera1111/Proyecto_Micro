; Programa de prueba simple para el Planner Lookahead
; Este programa permite verificar el funcionamiento del buffer progresivo

; Verificar estado inicial
PLANNER_STATUS

; Homing
G28

; Movimientos básicos para llenar y vaciar el buffer gradualmente
G0 X0 Y0 Z2

; Cuadrado simple con planner
G1 X10 Y0 F100
G1 X10 Y10 F100  
G1 X0 Y10 F100
G1 X0 Y0 F100

; Arco simple
G2 X5 Y5 R5 F80

; Vuelta al origen
G0 X0 Y0

; Sincronizar
PLANNER_SYNC

; Estado final
PLANNER_STATUS
