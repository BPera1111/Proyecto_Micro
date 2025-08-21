# Scripts PowerShell para ESP32 CNC Bridge
# Uso: Ejecuta estos comandos en PowerShell desde la carpeta uart_esp32

# ======================================
# INSTALACIÓN Y CONFIGURACIÓN
# ======================================

# 1. Instalar dependencias Python
function Install-PythonDeps {
    Write-Host "📦 Instalando dependencias Python..." -ForegroundColor Green
    pip install -r requirements.txt
}

# 2. Verificar IP del ESP32 en la red
function Find-ESP32 {
    param([string]$NetworkBase = "192.168.1")
    
    Write-Host "🔍 Buscando ESP32 en red $NetworkBase.x..." -ForegroundColor Yellow
    
    1..254 | ForEach-Object -Parallel {
        $ip = "$using:NetworkBase.$_"
        if (Test-Connection -ComputerName $ip -Count 1 -Quiet -TimeoutSeconds 1) {
            try {
                $response = Invoke-WebRequest -Uri "http://$ip/status" -TimeoutSec 2 -ErrorAction SilentlyContinue
                if ($response.StatusCode -eq 200 -and $response.Content -like "*uart_baud*") {
                    Write-Host "✅ ESP32 encontrado en: $ip" -ForegroundColor Green
                }
            } catch {
                # No es nuestro ESP32
            }
        }
    } -ThrottleLimit 20
}

# 3. Probar conectividad
function Test-ESP32Connection {
    param([string]$IP)
    
    if (-not $IP) {
        $IP = Read-Host "Ingresa la IP del ESP32"
    }
    
    Write-Host "🧪 Probando ESP32 en $IP..." -ForegroundColor Cyan
    python test_bridge.py $IP
}

# 4. Cliente interactivo
function Start-CNCClient {
    param([string]$IP)
    
    if (-not $IP) {
        $IP = Read-Host "Ingresa la IP del ESP32"
    }
    
    Write-Host "🎮 Iniciando cliente CNC interactivo..." -ForegroundColor Green
    python esp32_client.py $IP
}

# 5. Enviar comando único
function Send-CNCCommand {
    param(
        [string]$IP,
        [string]$Command
    )
    
    if (-not $IP) {
        $IP = Read-Host "Ingresa la IP del ESP32"
    }
    
    if (-not $Command) {
        $Command = Read-Host "Ingresa el comando G-code"
    }
    
    Write-Host "📤 Enviando: $Command" -ForegroundColor Yellow
    python esp32_client.py $IP $Command
}

# 6. Abrir interfaz web
function Open-CNCWeb {
    param([string]$IP)
    
    if (-not $IP) {
        $IP = Read-Host "Ingresa la IP del ESP32"
    }
    
    $url = "http://$IP"
    Write-Host "🌐 Abriendo $url en navegador..." -ForegroundColor Green
    Start-Process $url
}

# 7. Monitor de log en tiempo real
function Watch-CNCLog {
    param([string]$IP)
    
    if (-not $IP) {
        $IP = Read-Host "Ingresa la IP del ESP32"
    }
    
    Write-Host "📺 Monitoreando CNC en tiempo real... (Ctrl+C para salir)" -ForegroundColor Cyan
    python esp32_client.py $IP
}

# 8. Comandos de prueba comunes
function Test-CNCCommands {
    param([string]$IP)
    
    if (-not $IP) {
        $IP = Read-Host "Ingresa la IP del ESP32"
    }
    
    Write-Host "🧪 Ejecutando comandos de prueba..." -ForegroundColor Yellow
    
    $commands = @("G21", "G90", "M114", "G28", "M105")
    
    foreach ($cmd in $commands) {
        Write-Host "Enviando: $cmd" -ForegroundColor Cyan
        python esp32_client.py $IP $cmd
        Start-Sleep -Seconds 2
    }
}

# ======================================
# EJEMPLOS DE USO
# ======================================

Write-Host @"
🚀 ESP32 CNC Bridge - Comandos PowerShell

📋 FUNCIONES DISPONIBLES:
  Install-PythonDeps          - Instalar dependencias
  Find-ESP32                  - Buscar ESP32 en la red
  Test-ESP32Connection <IP>   - Probar conexión
  Start-CNCClient <IP>        - Cliente interactivo
  Send-CNCCommand <IP> <cmd>  - Enviar comando único
  Open-CNCWeb <IP>           - Abrir interfaz web
  Watch-CNCLog <IP>          - Monitor en tiempo real
  Test-CNCCommands <IP>      - Comandos de prueba

💡 EJEMPLOS:
  Install-PythonDeps
  Find-ESP32
  Test-ESP32Connection "192.168.1.100"
  Start-CNCClient "192.168.1.100"
  Send-CNCCommand "192.168.1.100" "G28"
  Open-CNCWeb "192.168.1.100"

🔧 COMANDOS G-CODE ÚTILES:
  G28        - Home (ir a origen)
  M114       - Posición actual
  G1 X10 Y10 - Mover a X10, Y10
  G21        - Modo milímetros
  G90        - Coordenadas absolutas
  M105       - Estado del sistema
  M84        - Desactivar motores

📱 USO RÁPIDO:
  1. Install-PythonDeps
  2. Find-ESP32 (para encontrar la IP)
  3. Test-ESP32Connection <IP_ENCONTRADA>
  4. Open-CNCWeb <IP> o Start-CNCClient <IP>

"@ -ForegroundColor Green
