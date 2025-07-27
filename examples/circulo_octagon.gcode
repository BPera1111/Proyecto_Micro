; Círculo aproximado usando líneas rectas
; Radio: 25mm, Centro en (25,25)

G28 ; Homing
G0 X50 Y25 Z5 ; Posición inicial
G1 Z0 F100 ; Bajar herramienta

; Octágono que aproxima un círculo
G1 X46.97 Y7.32 F300
G1 X35.36 Y-3.54 F300
G1 X21.47 Y-3.54 F300
G1 X9.86 Y7.32 F300
G1 X3.03 Y21.21 F300
G1 X3.03 Y35.10 F300
G1 X9.86 Y48.99 F300
G1 X21.47 Y60.60 F300
G1 X35.36 Y60.60 F300
G1 X46.97 Y48.99 F300
G1 X53.80 Y35.10 F300
G1 X53.80 Y21.21 F300
G1 X50 Y25 F300 ; Cerrar

G0 Z5 ; Subir herramienta
G0 X0 Y0 ; Volver al origen
