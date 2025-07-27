#!/usr/bin/env python3
"""
G-code Sender Simple - STM32 CNC
Script minimalista para enviar G-code predefinido
"""

import serial
import time

# =============================================================================
# CONFIGURACIÓN - MODIFICA ESTOS VALORES
# =============================================================================

# Puerto serie (cambia por el tuyo)
PUERTO = "COM13"  # En Windows: COM3, COM4, etc. | En Linux: /dev/ttyUSB0, /dev/ttyACM0

# G-code a enviar (una línea por string)
# Dirección del archivo G-code
GCODE_PATH = "examples/test_basico.gcode"

def cargar_gcode_desde_archivo(path):
    """Carga líneas de G-code desde un archivo"""
    with open(path, "r", encoding="utf-8") as f:
        lineas = [linea.strip() for linea in f if linea.strip() and not linea.strip().startswith(';')]
    return lineas

GCODE_PROGRAMA = cargar_gcode_desde_archivo(GCODE_PATH)

# =============================================================================
# FUNCIONES
# =============================================================================

def conectar():
    """Conecta al puerto serie"""
    try:
        ser = serial.Serial(
            port=PUERTO,
            baudrate=115200,
            timeout=5,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE
        )
        time.sleep(2)  # Esperar conexión USB CDC
        print(f"✅ Conectado a {PUERTO}")
        return ser
    except Exception as e:
        print(f"❌ Error de conexión: {e}")
        return None

def enviar_comando(ser, comando):
    """Envía un comando y espera respuesta"""
    try:
        # Enviar comando
        full_cmd = comando + '\r\n'
        ser.write(full_cmd.encode('utf-8'))
        print(f"📤 {comando}")
        
        # Esperar respuesta
        time.sleep(0.5)
        response = ""
        start_time = time.time()
        
        while (time.time() - start_time) < 3:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                response += data
                if "ok" in response.lower() or "error" in response.lower():
                    break
            time.sleep(0.1)
        
        print(f"📥 {response.strip()}")
        return True
        
    except Exception as e:
        print(f"❌ Error enviando comando: {e}")
        return False

def cargar_programa(ser, lineas_gcode):
    """Carga programa completo en el STM32"""
    print("\n🚀 Cargando programa...")
    
    # Iniciar modo almacenamiento
    if not enviar_comando(ser, "PROGRAM_START"):
        print("❌ Error al iniciar modo almacenamiento")
        return False
    
    # Enviar cada línea
    for i, linea in enumerate(lineas_gcode, 1):
        print(f"[{i}/{len(lineas_gcode)}]", end=" ")
        if not enviar_comando(ser, linea):
            print(f"⚠️ Error en línea {i}")
    
    # Finalizar almacenamiento
    enviar_comando(ser, "FIN")
    print("✅ Programa cargado")
    return True

def ejecutar_programa(ser):
    """Ejecuta el programa cargado"""
    print("\n🎯 Ejecutando programa...")
    enviar_comando(ser, "PROGRAM_RUN")

# =============================================================================
# PROGRAMA PRINCIPAL
# =============================================================================

def main():
    print("🔧 G-code Sender Simple")
    print("=" * 30)
    
    # Mostrar configuración
    print(f"Puerto: {PUERTO}")
    print(f"Líneas de G-code: {len(GCODE_PROGRAMA)}")
    print("\nPrograma a enviar:")
    for i, linea in enumerate(GCODE_PROGRAMA, 1):
        print(f"  {i}. {linea}")
    
    input("\nPresiona Enter para continuar...")
    
    # Conectar
    ser = conectar()
    if not ser:
        return
    
    try:
        # Cargar y ejecutar programa
        if cargar_programa(ser, GCODE_PROGRAMA):
            ejecutar_programa(ser)
        
        print("\n✅ Completado")
        
    except KeyboardInterrupt:
        print("\n⏹️ Interrumpido por el usuario")
    finally:
        if ser and ser.is_open:
            ser.close()
        print("🔌 Desconectado")

if __name__ == "__main__":
    main()
