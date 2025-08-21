#!/usr/bin/env python3
"""
Cliente Python para ESP32 UART Bridge
Permite controlar el CNC desde línea de comandos o script
"""

import asyncio
import websockets
import requests
import json
import threading
import sys
from datetime import datetime

class ESP32CNCClient:
    def __init__(self, esp32_ip, web_port=80, ws_port=81):
        self.esp32_ip = esp32_ip
        self.web_port = web_port
        self.ws_port = ws_port
        self.websocket = None
        self.connected = False
        self.responses = []
        
    async def connect(self):
        """Conectar al WebSocket del ESP32"""
        try:
            uri = f"ws://{self.esp32_ip}:{self.ws_port}"
            print(f"Conectando a {uri}...")
            
            self.websocket = await websockets.connect(uri)
            self.connected = True
            print("✅ Conectado al ESP32 CNC Bridge")
            
            # Iniciar listener en background
            asyncio.create_task(self.listen_responses())
            
        except Exception as e:
            print(f"❌ Error conectando: {e}")
            self.connected = False
            
    async def listen_responses(self):
        """Escuchar respuestas del CNC en background"""
        try:
            async for message in self.websocket:
                timestamp = datetime.now().strftime("%H:%M:%S")
                print(f"[{timestamp}] {message.strip()}")
                self.responses.append(message.strip())
                
        except websockets.exceptions.ConnectionClosed:
            print("⚠️ Conexión WebSocket cerrada")
            self.connected = False
        except Exception as e:
            print(f"❌ Error recibiendo datos: {e}")
            
    async def send_command(self, command):
        """Enviar comando al CNC"""
        if not self.connected or not self.websocket:
            print("❌ No conectado al ESP32")
            return False
            
        try:
            await self.websocket.send(command)
            print(f"📤 Enviado: {command}")
            return True
        except Exception as e:
            print(f"❌ Error enviando comando: {e}")
            return False
            
    def get_status(self):
        """Obtener estado del ESP32 via HTTP"""
        try:
            response = requests.get(f"http://{self.esp32_ip}:{self.web_port}/status", timeout=5)
            if response.status_code == 200:
                return json.loads(response.text)
            else:
                return None
        except Exception as e:
            print(f"❌ Error obteniendo estado: {e}")
            return None
            
    def send_command_http(self, command):
        """Enviar comando via HTTP (alternativo)"""
        try:
            data = {'command': command}
            response = requests.post(f"http://{self.esp32_ip}:{self.web_port}/send", 
                                   data=data, timeout=5)
            return response.status_code == 200
        except Exception as e:
            print(f"❌ Error enviando via HTTP: {e}")
            return False

async def interactive_mode(client):
    """Modo interactivo para enviar comandos"""
    print("\n🎮 Modo Interactivo CNC")
    print("Comandos especiales:")
    print("  'quit' o 'exit' - Salir")
    print("  'status' - Ver estado del ESP32")
    print("  'help' - Mostrar comandos comunes")
    print("  'clear' - Limpiar pantalla")
    print("\nEscribe comandos G-code para enviar al CNC:\n")
    
    while client.connected:
        try:
            command = input("CNC> ").strip()
            
            if command.lower() in ['quit', 'exit']:
                break
            elif command.lower() == 'status':
                status = client.get_status()
                if status:
                    print("📊 Estado ESP32:")
                    for key, value in status.items():
                        print(f"  {key}: {value}")
                else:
                    print("❌ No se pudo obtener estado")
            elif command.lower() == 'help':
                print_help()
            elif command.lower() == 'clear':
                import os
                os.system('cls' if os.name == 'nt' else 'clear')
            elif command:
                await client.send_command(command)
                
        except KeyboardInterrupt:
            break
        except EOFError:
            break
            
    print("\n👋 Saliendo del modo interactivo")

def print_help():
    """Mostrar comandos G-code comunes"""
    print("\n📖 Comandos G-code comunes:")
    print("  G28        - Home (ir a origen)")
    print("  M114       - Reportar posición actual")
    print("  G1 X10 Y10 - Mover a X10, Y10")
    print("  G1 X0 Y0   - Ir a origen")
    print("  M105       - Estado del sistema")
    print("  G21        - Modo milímetros")
    print("  G90        - Coordenadas absolutas")
    print("  G91        - Coordenadas relativas")
    print("  M84        - Desactivar motores")
    print("  G4 P1000   - Pausa 1 segundo")

async def batch_mode(client, commands):
    """Ejecutar lista de comandos en secuencia"""
    print(f"\n🔄 Ejecutando {len(commands)} comandos...")
    
    for i, command in enumerate(commands, 1):
        if not client.connected:
            print("❌ Conexión perdida")
            break
            
        print(f"[{i}/{len(commands)}] Ejecutando: {command}")
        await client.send_command(command)
        
        # Pausa entre comandos
        await asyncio.sleep(0.5)
        
    print("✅ Batch completado")

async def main():
    if len(sys.argv) < 2:
        print("❌ Uso: python esp32_client.py <IP_ESP32> [comandos...]")
        print("Ejemplos:")
        print("  python esp32_client.py 192.168.1.100")
        print("  python esp32_client.py 192.168.1.100 G28 'G1 X10 Y10' M114")
        return
        
    esp32_ip = sys.argv[1]
    commands = sys.argv[2:] if len(sys.argv) > 2 else None
    
    # Crear cliente
    client = ESP32CNCClient(esp32_ip)
    
    # Verificar conectividad
    print(f"🔍 Verificando ESP32 en {esp32_ip}...")
    status = client.get_status()
    if not status:
        print("❌ No se puede conectar al ESP32. Verifica:")
        print("  - IP correcta")
        print("  - ESP32 encendido y conectado a WiFi")
        print("  - PC en la misma red")
        return
        
    print("✅ ESP32 respondiendo")
    print(f"📊 Estado: WiFi={status.get('wifi_connected')}, Cliente={status.get('client_connected')}")
    
    # Conectar WebSocket
    await client.connect()
    
    if not client.connected:
        print("❌ No se pudo establecer conexión WebSocket")
        return
        
    # Modo de operación
    if commands:
        # Modo batch
        await batch_mode(client, commands)
    else:
        # Modo interactivo
        await interactive_mode(client)
        
    # Cerrar conexión
    if client.websocket:
        await client.websocket.close()
        
    print("👋 Desconectado")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n👋 Programa interrumpido por usuario")
    except Exception as e:
        print(f"❌ Error: {e}")
