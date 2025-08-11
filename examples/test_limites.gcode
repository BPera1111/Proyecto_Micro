; Test de límites de software para CNC
; Máquina con límites: X[0-160], Y[0-160], Z[0-160] mm
; 
; Comandos que DEBEN funcionar (dentro de límites)
G28         
G0 X0 Y0 Z0 
G0 X160 Y160 Z160
G0 X80 Y80 Z80   
G1 X100 Y100 Z100 F200 

; Comandos que DEBEN fallar (fuera de límites)
G0 X-10 Y0 Z0     
G0 X200 Y0 Z0     
G0 X0 Y-5 Z0      
G0 X0 Y200 Z0     
G0 X0 Y0 Z-1      
G0 X0 Y0 Z200     

; Test de límites múltiples
G0 X-10 Y200 Z-5  

; Finalizar en posición segura
G0 X50 Y50 Z10    
M5                
