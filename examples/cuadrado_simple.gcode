; Ejemplo de programa G-code simple
; Cuadrado de 50x50mm

G28 ; Homing
G0 X0 Y0 Z5 ; Posición inicial
G1 Z0 F100 ; Bajar herramienta
G1 X50 Y0 F500 ; Línea derecha
G1 X50 Y50 F500 ; Línea arriba
G1 X0 Y50 F500 ; Línea izquierda
G1 X0 Y0 F500 ; Línea abajo
G0 Z5 ; Subir herramienta
G0 X0 Y0 ; Volver al origen
