# ESP32 UART Bridge para CNC Monitor

Este proyecto convierte un ESP32 en un puente UART/WiFi para monitorear tu CNC STM32 sin necesidad de cable USB-TTL.

## 🔧 Hardware Necesario

- ESP32 (cualquier modelo con WiFi)
- Cables jumper para conexiones
- Protoboard (opcional pero recomendado)

## 📋 Conexiones

```
ESP32          STM32 (CNC)
GPIO16 (RX2) ←→ TX (PA9 o según tu configuración)
GPIO17 (TX2) ←→ RX (PA10 o según tu configuración)  
GND          ←→ GND (¡MUY IMPORTANTE!)
```

**⚠️ IMPORTANTE:**
- NO conectes VCC del ESP32 al STM32 si ambos tienen alimentación independiente
- Asegúrate de conectar las tierras (GND) entre ambos dispositivos
- Verifica que los niveles de voltaje sean compatibles (ambos trabajan a 3.3V)

## 🚀 Instalación del Software

### 1. Preparar Arduino IDE

1. Instala Arduino IDE (versión 1.8.19 o superior)
2. Agrega el soporte para ESP32:
   - Ve a `Archivo > Preferencias`
   - En "URLs adicionales de gestor de tarjetas" agrega:
     ```
     https://dl.espressif.com/dl/package_esp32_index.json
     ```
   - Ve a `Herramientas > Placa > Gestor de tarjetas`
   - Busca "ESP32" e instala "ESP32 by Espressif Systems"

### 2. Instalar Librerías Necesarias

Ve a `Herramientas > Administrar bibliotecas` e instala:

- **WebSockets** by Markus Sattler (versión 2.3.6 o superior)
  ```
  Buscar: "WebSockets Markus"
  ```

### 3. Configurar el Código

1. Abre `uart_bridge.ino` en Arduino IDE
2. **MODIFICA estas líneas con tu configuración WiFi:**
   ```cpp
   const char* ssid = "TU_RED_WIFI";        // ← Cambia por tu red WiFi
   const char* password = "TU_PASSWORD";    // ← Cambia por tu contraseña
   ```

3. **Verifica la configuración UART:**
   ```cpp
   #define STM32_UART_BAUD 115200  // Velocidad igual que tu STM32
   #define STM32_RX_PIN 16         // Pin RX del ESP32
   #define STM32_TX_PIN 17         // Pin TX del ESP32
   ```

### 4. Subir el Código

1. Conecta tu ESP32 por USB
2. Selecciona la placa correcta en `Herramientas > Placa`
3. Selecciona el puerto COM correcto
4. Haz clic en "Subir" (flecha hacia la derecha)

## 📱 Uso del Sistema

### 1. Primera Configuración

1. Después de subir el código, abre el Monitor Serie (115200 baud)
2. Reinicia el ESP32 y verás algo como:
   ```
   === ESP32 CNC UART Bridge ===
   Conectando a WiFi....
   WiFi conectado!
   IP: 192.168.1.100
   Sistema listo!
   ```

### 2. Acceso Web

1. Abre tu navegador web
2. Ve a la IP que mostró el ESP32 (ej: `http://192.168.1.100`)
3. Verás la interfaz web del monitor CNC

### 3. Funciones Disponibles

- **Monitor en tiempo real:** Ver todos los mensajes del STM32
- **Envío de comandos:** Escribir G-code y enviarlo al CNC
- **Comandos rápidos:** Botones para comandos comunes (Home, Status, etc.)
- **Interfaz responsive:** Funciona en PC, tablet y móvil

## 🎮 Comandos de Prueba

Una vez conectado, prueba estos comandos básicos:

```gcode
G28          ; Home (ir a origen)
M114         ; Reportar posición actual  
G1 X10 Y10   ; Mover a posición X10, Y10
M105         ; Estado del sistema
G21          ; Modo milímetros
G90          ; Modo coordenadas absolutas
```

## 🔍 Resolución de Problemas

### No conecta a WiFi
- Verifica SSID y contraseña
- Asegúrate que el ESP32 esté cerca del router
- Revisa el Monitor Serie para ver errores

### No recibe datos del STM32
- Verifica las conexiones TX/RX (pueden estar cruzadas)
- Confirma que las velocidades UART coincidan
- Asegúrate que GND esté conectado

### Página web no carga
- Verifica la IP en el Monitor Serie
- Asegúrate que tu PC esté en la misma red WiFi
- Prueba reiniciar el ESP32

### Comandos no llegan al STM32
- Revisa las conexiones físicas
- Confirma que el STM32 esté configurado para UART
- Verifica la velocidad de baudios

## 📊 Ventajas de esta Solución

✅ **Sin cables USB:** Monitoreo inalámbrico  
✅ **Interfaz web:** No necesitas software especial  
✅ **Multiplataforma:** Funciona en PC, móvil, tablet  
✅ **Tiempo real:** Comunicación bidireccional instantánea  
✅ **Comandos rápidos:** Botones para operaciones comunes  
✅ **Historial:** Ve todo el log de comunicación  
✅ **Portable:** Accede desde cualquier dispositivo en la red  

## 🔄 Actualizaciones

Para modificar el código:
1. Edita `uart_bridge.ino`
2. Vuelve a subir al ESP32
3. El ESP32 mantendrá la nueva configuración

## 🤝 Soporte

Si tienes problemas:
1. Revisa las conexiones físicas
2. Verifica la configuración WiFi
3. Consulta el Monitor Serie para errores
4. Prueba con comandos simples primero

¡Disfruta tu monitor CNC inalámbrico! 🎉
