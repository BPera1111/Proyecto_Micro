# Diagrama de Conexiones ESP32 ↔ STM32

```
     ESP32                           STM32F103 (CNC)
   ┌─────────┐                     ┌─────────────────┐
   │         │                     │                 │
   │ GPIO16  │◄────────────────────│ PA9 (TX)        │
   │ (RX2)   │     Datos del CNC   │                 │
   │         │                     │                 │
   │ GPIO17  │─────────────────────►│ PA10 (RX)       │
   │ (TX2)   │   Comandos al CNC   │                 │
   │         │                     │                 │
   │ GND     │─────────────────────│ GND             │
   │         │   ¡IMPORTANTE!      │                 │
   │ 3.3V    │ ◄─┐             ┌─► │ 3.3V            │
   │ (No     │   │ Alimentación│   │ (No conectar    │
   │ conectar│   │ independiente   │ si ambos tienen │
   │ si tiene│   │             │   │ alimentación    │
   │ aliment.)   │             │   │ propia)         │
   └─────────┘   │             │   └─────────────────┘
       │         │             │
       │    ┌────▼───┐     ┌────▼───┐
       │    │ USB/DC │     │ ST-Link│
       │    │ Power  │     │ or USB │
       │    └────────┘     └────────┘
       │
   ┌───▼─────┐
   │ WiFi    │ ◄─── Tu PC/Móvil
   │ Router  │      (192.168.1.X)
   └─────────┘
```

## Configuración de Pines STM32

Si tu STM32 usa otros pines para UART, modifica estas líneas en el código ESP32:

```cpp
// Para UART1 (PA9/PA10) - Configuración más común
#define STM32_RX_PIN 16  // ESP32 RX -> STM32 TX (PA9)
#define STM32_TX_PIN 17  // ESP32 TX -> STM32 RX (PA10)

// Para UART2 (PA2/PA3) - Si usas esta configuración
// Cambia las conexiones físicas en consecuencia
```

## Verificación de Conexiones

### Paso 1: Verifica STM32
1. Con un cable USB-TTL temporal, confirma que tu STM32 envía datos por UART
2. Anota qué pins usas (PA9/PA10, PA2/PA3, etc.)
3. Verifica la velocidad (normalmente 115200)

### Paso 2: Conexiones ESP32
```
ESP32 Pin    →    STM32 Pin    →    Función
GPIO16       →    PA9 (o TX)   →    Recibir datos
GPIO17       →    PA10 (o RX)  →    Enviar comandos  
GND          →    GND          →    Referencia común
```

### Paso 3: Prueba sin STM32
1. Conecta GPIO16 y GPIO17 del ESP32 entre sí (loopback)
2. Lo que envíes por WebSocket debería aparecer de vuelta
3. Esto confirma que el ESP32 funciona correctamente

## Niveles de Voltaje
- ✅ ESP32: 3.3V logic (compatible)
- ✅ STM32F103: 3.3V logic (compatible)
- ⚠️ Si usas Arduino (5V), necesitas divisor de voltaje o level shifter

## Troubleshooting Conexiones

### Problema: No llegan datos del STM32
**Solución:**
1. Intercambia GPIO16 y GPIO17 (TX/RX cruzados)
2. Verifica GND conectado
3. Confirma velocidad UART (115200)

### Problema: Comandos no llegan al STM32
**Solución:**
1. Verifica que STM32 esté configurado para recibir por UART
2. Confirma que usas los pines correctos
3. Prueba con terminal directa al STM32

### Problema: Datos corruptos
**Solución:**
1. Verifica velocidades iguales en ambos lados
2. Confirma conexión GND
3. Revisa integridad de cables

## Alimentación

### Opción 1: Alimentación separada (RECOMENDADO)
- ESP32: USB o fuente externa
- STM32: ST-Link, USB o fuente externa
- Solo conectar: GPIO16, GPIO17, GND

### Opción 2: Alimentación compartida
- Una fuente 3.3V para ambos
- Conectar VCC además de señales
- ⚠️ Cuidado con consumo de corriente
