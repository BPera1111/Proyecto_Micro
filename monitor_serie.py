#!/usr/bin/env python3
"""
Monitor Serie para CNC Controller
================================

Monitor serie simple para comunicarse con la Blue Pill STM32F103.
Soporte para comandos especiales que empiecen con "q:" para funciones Python.

Comandos Python:
- q:quit / q:exit   : Salir del programa
- q:ports           : Listar puertos COM disponibles
- q:connect COMx    : Conectar a puerto específico
- q:disconnect      : Desconectar puerto actual
- q:load archivo.gcode : Cargar archivo G-code línea por línea
- q:help            : Mostrar esta ayuda

Uso:
1. Ejecutar el script
2. Conectar con q:connect COM3 (o el puerto que uses)
3. Enviar comandos G-code normalmente
4. Usar comandos q: para funciones especiales

Autor: Bruno Pera
Fecha: Agosto 2025
"""

import serial
import serial.tools.list_ports
import time
import os
import sys
import threading

class MonitorSerie:
    def __init__(self):
        self.serial_port = None
        self.connected = False
        self.running = True
        
    def listar_puertos(self):
        """Lista todos los puertos COM disponibles"""
        puertos = serial.tools.list_ports.comports()
        if puertos:
            print("\n=== Puertos COM Disponibles ===")
            for puerto in puertos:
                print(f"  {puerto.device}: {puerto.description}")
            print("================================\n")
        else:
            print("No se encontraron puertos COM disponibles.\n")
            
    def conectar(self, puerto, baudrate=115200):
        """Conecta al puerto serie especificado"""
        try:
            if self.connected:
                print(f"Ya estás conectado a {self.serial_port.port}. Desconecta primero.\n")
                return False
                
            self.serial_port = serial.Serial(puerto, baudrate, timeout=1)
            self.connected = True
            print(f"✓ Conectado a {puerto} @ {baudrate} bps\n")
            
            # Iniciar hilo de lectura
            self.hilo_lectura = threading.Thread(target=self.leer_datos, daemon=True)
            self.hilo_lectura.start()
            
            return True
        except Exception as e:
            print(f"✗ Error al conectar a {puerto}: {e}\n")
            return False
            
    def desconectar(self):
        """Desconecta del puerto serie"""
        if self.connected and self.serial_port:
            self.connected = False
            self.serial_port.close()
            print("✓ Puerto desconectado\n")
        else:
            print("No hay conexión activa\n")
            
    def leer_datos(self):
        """Hilo para leer datos del puerto serie"""
        while self.connected and self.serial_port:
            try:
                if self.serial_port.in_waiting > 0:
                    data = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                    if data:
                        print(f"← {data}")
            except Exception as e:
                if self.connected:  # Solo mostrar error si todavía deberíamos estar conectados
                    print(f"Error de lectura: {e}")
                break
            time.sleep(0.01)
            
    def enviar_comando(self, comando):
        """Envía un comando al puerto serie"""
        if not self.connected:
            print("✗ No hay conexión activa. Conecta primero con q:connect COMx\n")
            return
            
        try:
            # Agregar salto de línea si no lo tiene
            if not comando.endswith('\n'):
                comando += '\n'
            
            self.serial_port.write(comando.encode('utf-8'))
            print(f"→ {comando.strip()}")
        except Exception as e:
            print(f"✗ Error al enviar comando: {e}\n")
            
    def cargar_gcode(self, archivo=None):
        """Carga un archivo G-code línea por línea"""
        if not self.connected:
            print("✗ No hay conexión activa. Conecta primero.\n")
            return
            
        try:
            # Si no se especifica archivo, mostrar menú de selección
            if archivo is None:
                gcode_dir = "examples"
                if not os.path.exists(gcode_dir):
                    print(f"✗ Directorio '{gcode_dir}' no encontrado.\n")
                    return
                    
                # Listar archivos .gcode disponibles
                archivos = [f for f in os.listdir(gcode_dir) if f.lower().endswith('.gcode')]
                if not archivos:
                    print(f"✗ No se encontraron archivos .gcode en '{gcode_dir}'\n")
                    return
                
                # Mostrar menú
                print(f"\n=== Archivos G-code disponibles en '{gcode_dir}' ===")
                for idx, nombre in enumerate(archivos, 1):
                    print(f"  {idx}. {nombre}")
                print("=" * 50)
                
                # Pedir selección
                try:
                    opcion = input(f"Selecciona archivo (1-{len(archivos)}) o Enter para cancelar: ").strip()
                    if not opcion:
                        print("Operación cancelada.\n")
                        return
                        
                    opcion = int(opcion)
                    if 1 <= opcion <= len(archivos):
                        archivo = os.path.join(gcode_dir, archivos[opcion - 1])
                    else:
                        print("✗ Opción inválida.\n")
                        return
                except ValueError:
                    print("✗ Ingresa un número válido.\n")
                    return
            
            # Cargar el archivo seleccionado
            with open(archivo, 'r', encoding='utf-8') as f:
                todas_las_lineas = f.readlines()
                
            # Filtrar líneas (igual que gcode.py)
            lineas_gcode = []
            for linea in todas_las_lineas:
                linea = linea.strip()
                if linea and not linea.startswith(';'):  # Omitir líneas vacías y comentarios
                    lineas_gcode.append(linea)
            
            if not lineas_gcode:
                print(f"✗ No hay comandos G-code válidos en {archivo}\n")
                return
                
            # Mostrar resumen
            print(f"\n=== Cargando {os.path.basename(archivo)} ===")
            print(f"Líneas de G-code: {len(lineas_gcode)}")
            print("Programa a enviar:")
            for i, linea in enumerate(lineas_gcode, 1):
                print(f"  {i}. {linea}")
            
            # Confirmar carga
            confirmar = input(f"\n¿Cargar programa de {len(lineas_gcode)} líneas? (S/n): ").strip().lower()
            if confirmar and confirmar not in ['s', 'si', 'y', 'yes', '']:
                print("Operación cancelada.\n")
                return
            
            # Iniciar modo almacenamiento (como gcode.py)
            print("\n🚀 Cargando programa...")
            if not self.enviar_comando_con_respuesta("PROGRAM_START"):
                print("✗ Error al iniciar modo almacenamiento\n")
                return
                
            # Enviar cada línea
            errores = 0
            for i, linea in enumerate(lineas_gcode, 1):
                print(f"[{i:3d}/{len(lineas_gcode)}] ", end="")
                if not self.enviar_comando_con_respuesta(linea):
                    print(f"⚠️ Error en línea {i}")
                    errores += 1
                
            # Finalizar almacenamiento
            self.enviar_comando_con_respuesta("FIN")
            
            if errores == 0:
                print("✅ Programa cargado exitosamente")
                
                # Preguntar si ejecutar
                ejecutar = input("¿Ejecutar programa ahora? (S/n): ").strip().lower()
                if not ejecutar or ejecutar in ['s', 'si', 'y', 'yes']:
                    print("\n🎯 Ejecutando programa...")
                    self.enviar_comando_con_respuesta("PROGRAM_RUN")
            else:
                print(f"⚠️ Programa cargado con {errores} errores")
                
            print("=" * 50 + "\n")
            
        except FileNotFoundError:
            print(f"✗ Archivo no encontrado: {archivo}\n")
        except Exception as e:
            print(f"✗ Error al cargar archivo: {e}\n")
            
    def enviar_comando_con_respuesta(self, comando):
        """Envía un comando y espera respuesta (como gcode.py)"""
        if not self.connected:
            return False
            
        try:
            # Enviar comando
            full_cmd = comando + '\r\n'
            self.serial_port.write(full_cmd.encode('utf-8'))
            print(f"📤 {comando}")
            
            # Esperar respuesta
            time.sleep(0.5)
            response = ""
            start_time = time.time()
            
            while (time.time() - start_time) < 3:
                if self.serial_port.in_waiting > 0:
                    data = self.serial_port.read(self.serial_port.in_waiting).decode('utf-8', errors='ignore')
                    response += data
                    if "ok" in response.lower() or "error" in response.lower():
                        break
                time.sleep(0.1)
            
            print(f"📥 {response.strip()}")
            return True if "error" not in response.lower() else False
            
        except Exception as e:
            print(f"✗ Error enviando comando: {e}")
            return False
            
    def mostrar_ayuda(self):
        """Muestra la ayuda de comandos"""
        print("""
=== MONITOR SERIE CNC ===

Comandos Python (empiezan con q:):
  q:quit, q:exit     - Salir del programa
  q:ports            - Listar puertos COM disponibles  
  q:connect COMx     - Conectar a puerto (ej: q:connect COM3)
  q:disconnect       - Desconectar puerto actual
  q:load             - Mostrar menú de archivos G-code disponibles
  q:load archivo     - Cargar archivo G-code específico
  q:help             - Mostrar esta ayuda

Comandos G-code (se envían directamente):
  G28                - Homing
  G0 X10 Y10         - Movimiento rápido
  G1 X20 Y20 F200    - Movimiento lineal
  M114               - Reportar posición
  M503               - Mostrar configuración
  M505               - Mostrar límites

Ejemplos:
  q:connect COM3
  G28
  G0 X50 Y50
  q:load
  q:load examples/test_basico.gcode

========================
""")

    def procesar_comando_python(self, comando):
        """Procesa comandos que empiezan con q:"""
        comando = comando[2:].strip()  # Quitar "q:"
        partes = comando.split()
        
        if not partes:
            return
            
        cmd = partes[0].lower()
        
        if cmd in ['quit', 'exit']:
            print("Saliendo...\n")
            self.running = False
            if self.connected:
                self.desconectar()
                
        elif cmd == 'ports':
            self.listar_puertos()
            
        elif cmd == 'connect':
            if len(partes) >= 2:
                puerto = partes[1].upper()
                if not puerto.startswith('COM'):
                    puerto = 'COM' + puerto
                self.conectar(puerto)
            else:
                print("Uso: q:connect COMx (ej: q:connect COM3)\n")
                
        elif cmd == 'disconnect':
            self.desconectar()
            
        elif cmd == 'load':
            if len(partes) >= 2:
                archivo = ' '.join(partes[1:])  # Permitir espacios en nombres
                self.cargar_gcode(archivo)
            else:
                # Sin archivo especificado, mostrar menú
                self.cargar_gcode()
                
        elif cmd == 'help':
            self.mostrar_ayuda()
            
        else:
            print(f"Comando desconocido: {cmd}. Usa q:help para ver comandos disponibles.\n")
            
    def ejecutar(self):
        """Bucle principal del monitor"""
        print("=== Monitor Serie CNC ===")
        print("Usa q:help para ver comandos disponibles")
        print("Usa q:ports para ver puertos COM")
        print("Usa q:connect COMx para conectar\n")
        
        try:
            while self.running:
                try:
                    entrada = input("> ").strip()
                    
                    if not entrada:
                        continue
                        
                    if entrada.startswith('q:'):
                        self.procesar_comando_python(entrada)
                    else:
                        self.enviar_comando(entrada)
                        
                except KeyboardInterrupt:
                    print("\n\nInterrumpido por usuario. Saliendo...\n")
                    break
                except EOFError:
                    print("\nSaliendo...\n")
                    break
                    
        finally:
            if self.connected:
                self.desconectar()

if __name__ == "__main__":
    monitor = MonitorSerie()
    monitor.ejecutar()