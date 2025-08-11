; Test de violación de límites durante la carga
; Este archivo DEBE fallar durante la carga, no durante la ejecución
;
; Programa con comandos válidos
G28         ; Home (válido)
G0 X10 Y10 Z10  ; Movimiento válido

; Comando que viola límites - DEBE detener la carga aquí
G0 X200 Y50 Z50  ; X=200 está fuera del límite (max 160)

; Estos comandos NO deben ser cargados debido al error anterior
G0 X50 Y50 Z50   ; Este comando es válido pero no debe ejecutarse
G1 X100 Y100 F200 ; Este comando es válido pero no debe ejecutarse
M5               ; Este comando es válido pero no debe ejecutarse
