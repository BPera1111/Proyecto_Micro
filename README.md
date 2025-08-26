# 🔧 CNC Láser Controller - STM32F103C8T6

Proyecto de conversión de impresora 3D a CNC láser utilizando microcontrolador STM32F103C8T6 (Blue Pill).

## 📋 Características

- **Control de 3 ejes** (X, Y, Z) con motores paso a paso
- **Comunicación USB CDC** para comandos G-code
- **Finales de carrera** en los 3 ejes
- **LEDs indicadores** de dirección de movimiento
- **Procesamiento G-code básico** (G0, G1)

## 🔧 Hardware Requerido

- **STM32F103C8T6** (Blue Pill)
- **ST-Link V2** (para programación y debug)
- **3x Drivers de motores paso a paso** (A4988/DRV8825)
- **3x Motores paso a paso NEMA17**
- **3x Finales de carrera** (endstops)
- **2x LEDs** (indicadores de dirección)

## 📍 Configuración de Pines

| Función | Pin STM32 | Descripción |
|---------|-----------|-------------|
| **Motor X** | | |
| X_STEP | PB6 | Pulsos de paso |
| X_DIR | PB7 | Dirección |
| X_EN | PB8 | Habilitación |
| X_MIN | PB12 | Final de carrera |
| **Motor Y** | | |
| Y_STEP | PB9 | Pulsos de paso |
| Y_DIR | PB3 | Dirección |
| Y_EN | PB4 | Habilitación |
| Y_MIN | PB13 | Final de carrera |
| **Motor Z** | | |
| Z_STEP | PA8 | Pulsos de paso |
| Z_DIR | PA9 | Dirección |
| Z_EN | PA10 | Habilitación |
| Z_MIN | PB14 | Final de carrera |
| **LEDs** | | |
| LED_HORARIO | PB0 | Indicador sentido horario |
| LED_ANTIHORARIO | PB1 | Indicador sentido antihorario |

## 🚀 Instalación y Configuración

### 1. Preparación del entorno

```bash
# Instalar pyOCD para programación
pip install pyocd

# Verificar dispositivos conectados
pyocd list
```

### 2. Programación del firmware

```bash
# Navegar al directorio Debug generado por STM32CubeIDE
cd Proyecto_Micro/CNC/Debug

# Flashear el firmware (borra chip completo)
pyocd flash --target stm32f103rc --erase=chip CNC.elf
```

## 🐛 Debug y Desarrollo

### Configuración para VSCode

1. **Instalar extensión:** `Cortex-Debug` en VSCode

2. **Crear archivo de configuración:** `.vscode/launch.json`

```json
{
  "version": "0.2.0",
  "configurations": [
    {
      "name": "Debug STM32F103 via pyOCD",
      "type": "cortex-debug",
      "request": "launch",
      "executable": "${workspaceFolder}/CNC/Debug/CNC.elf",
      "servertype": "external",
      "gdbPath": "C:/ST/STM32CubeIDE_1.19.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.0.202411081344/tools/bin/arm-none-eabi-gdb.exe",
      "armToolchainPath": "C:/ST/STM32CubeIDE_1.19.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.0.202411081344/tools/bin",
      "gdbTarget": "localhost:3333",
      "runToEntryPoint": "main",
      "device": "STM32F103C8"
    }
  ]
}
```

3. **Iniciar servidor GDB:**

```bash
# En terminal separado
pyocd gdbserver --target stm32f103rc
```

4. **Debuggear:** Usar el menú de depuración de VSCode seleccionando la configuración "Debug STM32F103 via pyOCD"

## 🎮 Comandos G-code Soportados

| Comando | Descripción | Ejemplo |
|---------|-------------|---------|
| `G0 Xnn Ynn Znn` | Movimiento rápido | `G0 X100 Y50` |
| `G1 Xnn Ynn Znn` | Movimiento lineal | `G1 X50 Y25 Z10` |

### Ejemplos de uso:

```gcode
G1 X100        # Mover X a 100mm
G1 Y50         # Mover Y a 50mm
G1 X50 Y25     # Movimiento combinado XY
G1 X0 Y0 Z0    # Volver al origen
```

## ⚙️ Configuración del Sistema

- **Velocidad de pasos:** 800μs entre pasos
- **Resolución:** 2000 pasos/revolución
- **Frecuencia de loop:** 20Hz (cada 50ms)
- **Verificación endstops:** Cada 10ms
- **Comunicación:** USB CDC (puerto serie virtual)

## 🛠️ Desarrollo

### Estructura del proyecto:

```
Proyecto_Micro/
├── README.md
├── AnteProyecto/          # Documentación LaTeX
└── CNC/                   # Proyecto STM32CubeIDE
    ├── Core/
    │   ├── Inc/           # Headers
    │   └── Src/           # Código fuente
    ├── Drivers/           # HAL STM32
    ├── USB_DEVICE/        # Configuración USB CDC
    └── Debug/             # Archivos compilados
```

### Control de debug:

En `main.c`, línea 44:
```c
#define DEBUG_MESSAGES 1  // 1=activar, 0=desactivar mensajes debug
```

## 📝 Notas

- **Ajustar rutas:** Modificar las rutas en `launch.json` según tu instalación de STM32CubeIDE
- **Target device:** Verificar que el target `stm32f103rc` sea compatible con tu Blue Pill
- **Conexiones:** Asegurar conexiones correctas de ST-Link y USB CDC

## 🔮 Próximas funcionalidades

- [ ] Control de láser con PWM
- [ ] Comandos M3/M5 (encender/apagar láser)
- [ ] Interpolación lineal simultánea
- [ ] Homing automático
- [ ] Control de velocidad variable
- [ ] Sistema de coordenadas robusto
- [ ] UART con ESP32

---

**Autores:** BPera1111 y lautoping  
**Fecha:** Julio 2025