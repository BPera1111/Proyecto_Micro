#!/usr/bin/env python3
"""
Script de prueba para ESP32 UART Bridge
Verifica conectividad y funciones básicas
"""

import requests
import asyncio
import websockets
import json
import time
import sys

async def test_esp32_bridge(esp32_ip):
    """Prueba completa del ESP32 Bridge"""
    
    print("🧪 === PRUEBA ESP32 CNC BRIDGE ===\n")
    
    # Test 1: Conectividad HTTP
    print("1️⃣ Probando conectividad HTTP...")
    try:
        response = requests.get(f"http://{esp32_ip}", timeout=5)
        if response.status_code == 200:
            print("   ✅ Servidor web responde")
        else:
            print(f"   ⚠️ Servidor responde con código {response.status_code}")
    except Exception as e:
        print(f"   ❌ Error HTTP: {e}")
        return False
    
    # Test 2: API Status
    print("\n2️⃣ Probando API de estado...")
    try:
        response = requests.get(f"http://{esp32_ip}/status", timeout=5)
        if response.status_code == 200:
            status = json.loads(response.text)
            print("   ✅ API de estado funciona")
            print(f"   📊 WiFi conectado: {status.get('wifi_connected')}")
            print(f"   📊 Cliente WebSocket: {status.get('client_connected')}")
            print(f"   📊 IP: {status.get('ip_address')}")
            print(f"   📊 UART Baud: {status.get('uart_baud')}")
        else:
            print(f"   ❌ API responde con código {response.status_code}")
    except Exception as e:
        print(f"   ❌ Error API: {e}")
    
    # Test 3: WebSocket Connection
    print("\n3️⃣ Probando conexión WebSocket...")
    try:
        uri = f"ws://{esp32_ip}:81"
        async with websockets.connect(uri, timeout=5) as websocket:
            print("   ✅ WebSocket conectado")
            
            # Test 4: Envío de comando
            print("\n4️⃣ Probando envío de comando...")
            test_command = "M114"  # Comando para obtener posición
            await websocket.send(test_command)
            print(f"   📤 Enviado: {test_command}")
            
            # Test 5: Recepción de datos
            print("\n5️⃣ Esperando respuesta (5 segundos)...")
            try:
                response = await asyncio.wait_for(websocket.recv(), timeout=5.0)
                print(f"   📥 Recibido: {response}")
                print("   ✅ Comunicación bidireccional funciona")
            except asyncio.TimeoutError:
                print("   ⚠️ No se recibió respuesta del STM32 en 5 segundos")
                print("   💡 Esto es normal si el STM32 no está conectado o configurado")
                
    except Exception as e:
        print(f"   ❌ Error WebSocket: {e}")
        return False
    
    # Test 6: HTTP Command Send
    print("\n6️⃣ Probando envío de comando por HTTP...")
    try:
        data = {'command': 'G21'}  # Comando para modo milímetros
        response = requests.post(f"http://{esp32_ip}/send", data=data, timeout=5)
        if response.status_code == 200:
            print("   ✅ Comando enviado por HTTP")
            print(f"   📄 Respuesta: {response.text}")
        else:
            print(f"   ❌ Error enviando comando: {response.status_code}")
    except Exception as e:
        print(f"   ❌ Error HTTP POST: {e}")
    
    return True

async def test_loopback(esp32_ip):
    """Prueba loopback si GPIO16 y GPIO17 están conectados"""
    print("\n🔄 === PRUEBA LOOPBACK ===")
    print("💡 Para esta prueba, conecta GPIO16 y GPIO17 del ESP32 entre sí")
    
    try:
        uri = f"ws://{esp32_ip}:81"
        async with websockets.connect(uri, timeout=5) as websocket:
            
            test_messages = ["HOLA", "G28", "M114", "TEST123"]
            
            for msg in test_messages:
                print(f"📤 Enviando: {msg}")
                await websocket.send(msg)
                
                try:
                    response = await asyncio.wait_for(websocket.recv(), timeout=3.0)
                    if msg in response:
                        print(f"   ✅ Recibido: {response}")
                    else:
                        print(f"   📥 Recibido diferente: {response}")
                except asyncio.TimeoutError:
                    print("   ⚠️ Sin respuesta")
                
                await asyncio.sleep(1)
                
    except Exception as e:
        print(f"❌ Error en prueba loopback: {e}")

def print_diagnostic_info():
    """Imprime información de diagnóstico"""
    print("\n🔍 === INFORMACIÓN DE DIAGNÓSTICO ===")
    print("\n📋 Checklist de verificación:")
    print("□ ESP32 programado con uart_bridge.ino")
    print("□ ESP32 conectado a WiFi (revisar Monitor Serie)")
    print("□ PC en la misma red que ESP32")
    print("□ IP del ESP32 anotada desde Monitor Serie")
    print("□ Conexiones físicas ESP32 ↔ STM32:")
    print("  • GPIO16 (ESP32) → TX del STM32")
    print("  • GPIO17 (ESP32) → RX del STM32") 
    print("  • GND (ESP32) → GND del STM32")
    
    print("\n🔧 Comandos útiles para debug:")
    print("• Verificar IP: ping <IP_ESP32>")
    print("• Ver página web: http://<IP_ESP32>")
    print("• Cliente Python: python esp32_client.py <IP_ESP32>")
    print("• Monitor Serie Arduino: velocidad 115200")
    
    print("\n⚡ Comandos G-code de prueba:")
    print("• G28    - Home")
    print("• M114   - Posición actual")
    print("• G21    - Modo milímetros")
    print("• M105   - Estado")

async def main():
    print("🚀 ESP32 CNC Bridge - Script de Prueba\n")
    
    if len(sys.argv) != 2:
        print("❌ Uso: python test_bridge.py <IP_ESP32>")
        print("Ejemplo: python test_bridge.py 192.168.1.100")
        return
    
    esp32_ip = sys.argv[1]
    print(f"🎯 Probando ESP32 en IP: {esp32_ip}\n")
    
    # Prueba principal
    success = await test_esp32_bridge(esp32_ip)
    
    if success:
        print("\n✨ === RESULTADO ===")
        print("✅ ESP32 Bridge funciona correctamente")
        print("🌐 Abre tu navegador en: http://" + esp32_ip)
        print("🐍 Usa cliente Python: python esp32_client.py " + esp32_ip)
        
        # Preguntar por prueba loopback
        try:
            choice = input("\n❓ ¿Hacer prueba loopback? (GPIO16-GPIO17 conectados) [y/N]: ")
            if choice.lower() in ['y', 'yes', 's', 'si']:
                await test_loopback(esp32_ip)
        except KeyboardInterrupt:
            pass
            
    else:
        print("\n❌ === PROBLEMAS DETECTADOS ===")
        print_diagnostic_info()
    
    print("\n👋 Fin de las pruebas")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n👋 Prueba interrumpida por usuario")
    except Exception as e:
        print(f"\n❌ Error en las pruebas: {e}")
