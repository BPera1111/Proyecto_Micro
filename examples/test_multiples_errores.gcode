; Test múltiples violaciones de límites
; Diferentes tipos de errores para probar la validación
;
G28                   
G0 X0 Y0 Z0           

; Violación en X (debe fallar aquí)
G0 X-10 Y50 Z50       

; Estas líneas NO deben procesarse:
G0 X50 Y200 Z50       
G0 X50 Y50 Z-5        
G1 X300 Y300 Z300 F200 
