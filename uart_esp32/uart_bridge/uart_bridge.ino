/*
 * ESP32 UART Bridge para CNC Monitor
 * Actúa como puente entre STM32 (CNC) y PC vía WiFi/Serial
 * 
 * Conexiones físicas requeridas:
 * ESP32 GPIO16 (RX2) -> STM32F103 PA2 (USART2_TX)
 * ESP32 GPIO17 (TX2) -> STM32F103 PA3 (USART2_RX)
 * ESP32 GND         -> STM32F103 GND (IMPORTANTE!)
 * 
 * Configuración UART: 115200 baud, 8N1
 * 
 * Autor: Bridge para proyecto CNC
 * Fecha: Agosto 2025
 */

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

// Configuración WiFi - CAMBIA ESTOS VALORES
const char* ssid = "BRUNO-WIN";
const char* password = "pirum11111";

// Configuración UART para comunicación con STM32
#define STM32_UART_BAUD 115200
#define STM32_RX_PIN 16  // GPIO16 -> conectar a STM32F103 PA2 (USART2_TX)
#define STM32_TX_PIN 17  // GPIO17 -> conectar a STM32F103 PA3 (USART2_RX)

// Servidores
WebServer server(80);
WebSocketsServer webSocket(81);

// Buffers para datos
String serialBuffer = "";
bool clientConnected = false;

void setup() {
  // Inicializar UART0 (debug/monitor)
  Serial.begin(115200);
  
  // Inicializar UART2 para comunicación con STM32
  Serial2.begin(STM32_UART_BAUD, SERIAL_8N1, STM32_RX_PIN, STM32_TX_PIN);
  
  delay(1000);
  Serial.println("\n=== ESP32 CNC UART Bridge ===");
  Serial.println("UART2 Config:");
  Serial.print("- Baud: ");
  Serial.println(STM32_UART_BAUD);
  Serial.print("- RX Pin (ESP32): ");
  Serial.print(STM32_RX_PIN);
  Serial.println(" -> conectar a STM32 PA2 (TX)");
  Serial.print("- TX Pin (ESP32): ");
  Serial.print(STM32_TX_PIN);
  Serial.println(" -> conectar a STM32 PA3 (RX)");
  
  // Configurar WiFi
  setupWiFi();
  
  // Configurar servidor web
  setupWebServer();
  
  // Configurar WebSocket
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  
  Serial.println("Sistema listo!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.println("Puerto WebSocket: 81");
  Serial.println("Conecta tu navegador a: http://" + WiFi.localIP().toString());
}

void loop() {
  // Manejar conexiones
  server.handleClient();
  webSocket.loop();
  
  // Leer datos del STM32 y enviar por WebSocket
  readFromSTM32();
  
  // Pequeña pausa para no saturar
  delay(10);
}

void setupWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Conectando a WiFi");
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println();
  Serial.println("WiFi conectado!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

void setupWebServer() {
  // Página principal
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", getWebPage());
  });
  
  // API para enviar comandos
  server.on("/send", HTTP_POST, []() {
    if (server.hasArg("command")) {
      String command = server.arg("command");
      sendToSTM32(command);
      server.send(200, "text/plain", "Comando enviado: " + command);
    } else {
      server.send(400, "text/plain", "Error: falta parámetro 'command'");
    }
  });
  
  // Estado del sistema
  server.on("/status", HTTP_GET, []() {
    String status = "{\n";
    status += "  \"wifi_connected\": " + String(WiFi.status() == WL_CONNECTED ? "true" : "false") + ",\n";
    status += "  \"client_connected\": " + String(clientConnected ? "true" : "false") + ",\n";
    status += "  \"ip_address\": \"" + WiFi.localIP().toString() + "\",\n";
    status += "  \"uart_baud\": " + String(STM32_UART_BAUD) + "\n";
    status += "}";
    server.send(200, "application/json", status);
  });
  
  server.begin();
  Serial.println("Servidor web iniciado en puerto 80");
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("Cliente [%u] desconectado\n", num);
      clientConnected = false;
      break;
      
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("Cliente [%u] conectado desde %d.%d.%d.%d\n", 
                     num, ip[0], ip[1], ip[2], ip[3]);
        clientConnected = true;
        
        // Enviar mensaje de bienvenida
        webSocket.sendTXT(num, "ESP32 CNC Monitor conectado\n");
      }
      break;
      
    case WStype_TEXT:
      {
        String command = String((char*)payload);
        command.trim();
        Serial.printf("Comando recibido: %s\n", command.c_str());
        sendToSTM32(command);
      }
      break;
      
    default:
      break;
  }
}

void readFromSTM32() {
  while (Serial2.available()) {
    char c = Serial2.read();
    
    // Debug: mostrar byte recibido en hexadecimal también
    Serial.print(c);
    Serial.print(" [0x");
    Serial.print((uint8_t)c, HEX);
    Serial.print("] ");
    
    // Enviar por WebSocket si hay clientes conectados
    if (clientConnected) {
      String data = String(c);
      webSocket.broadcastTXT(data);
    }
  }
  
  // Debug: cada 5 segundos, verificar si hay datos pendientes
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 5000) {
    lastCheck = millis();
    Serial.println("\n[DEBUG] Checking UART2 status...");
    Serial.print("Available bytes: ");
    Serial.println(Serial2.available());
  }
}

void sendToSTM32(String command) {
  // Asegurar que termine con \n
  if (!command.endsWith("\n")) {
    command += "\n";
  }
  
  // Enviar al STM32
  Serial2.print(command);
  
  // Log local
  Serial.print("-> STM32: ");
  Serial.print(command);
  
  // Confirmar por WebSocket
  if (clientConnected) {
    webSocket.broadcastTXT("SENT: " + command);
  }
}

String getWebPage() {
  return R"html(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>ESP32 CNC Monitor</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background: #f0f0f0; }
        .container { max-width: 800px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }
        .header { text-align: center; color: #333; border-bottom: 2px solid #007bff; padding-bottom: 10px; margin-bottom: 20px; }
        .status { background: #e7f3ff; padding: 10px; border-radius: 5px; margin-bottom: 20px; }
        .terminal { background: #000; color: #00ff00; font-family: monospace; height: 400px; overflow-y: scroll; padding: 10px; border-radius: 5px; white-space: pre-wrap; }
        .input-area { margin-top: 10px; display: flex; gap: 10px; }
        input[type="text"] { flex: 1; padding: 10px; border: 1px solid #ddd; border-radius: 5px; font-family: monospace; }
        button { padding: 10px 20px; background: #007bff; color: white; border: none; border-radius: 5px; cursor: pointer; }
        button:hover { background: #0056b3; }
        .quick-commands { margin-top: 10px; }
        .quick-commands button { margin: 5px; background: #28a745; }
        .quick-commands button:hover { background: #1e7e34; }
        .connected { color: #28a745; }
        .disconnected { color: #dc3545; }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🔧 ESP32 CNC Monitor</h1>
            <p>Monitor y control remoto para CNC via WiFi</p>
        </div>
        
        <div class="status">
            <strong>Estado de conexión:</strong> <span id="connectionStatus" class="disconnected">Desconectado</span><br>
            <strong>Comandos enviados:</strong> <span id="commandCount">0</span><br>
            <strong>IP ESP32:</strong> <span id="ipAddress">Obteniendo...</span>
        </div>
        
        <div id="terminal" class="terminal">ESP32 CNC Monitor - Conectando...\n</div>
        
        <div class="input-area">
            <input type="text" id="commandInput" placeholder="Escribe comando G-code aquí (ej: G1 X10 Y10 F1000)" onkeypress="handleEnter(event)">
            <button onclick="sendCommand()">Enviar</button>
            <button onclick="clearTerminal()">Limpiar</button>
        </div>
        
        <div class="quick-commands">
            <strong>Comandos rápidos:</strong><br>
            <button onclick="sendQuickCommand('G28')">Home (G28)</button>
            <button onclick="sendQuickCommand('M114')">Posición (M114)</button>
            <button onclick="sendQuickCommand('M105')">Estado (M105)</button>
            <button onclick="sendQuickCommand('M84')">Desactivar Motores</button>
            <button onclick="sendQuickCommand('G1 X10 Y10 F1000')">Test Move</button>
            <button onclick="sendQuickCommand('G21')">Modo mm</button>
            <button onclick="sendQuickCommand('G90')">Modo Absoluto</button>
        </div>
    </div>

    <script>
        let ws;
        let commandCount = 0;
        
        function connectWebSocket() {
            const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
            ws = new WebSocket(protocol + '//' + window.location.hostname + ':81');
            
            ws.onopen = function() {
                document.getElementById('connectionStatus').textContent = 'Conectado';
                document.getElementById('connectionStatus').className = 'connected';
                addToTerminal('WebSocket conectado al ESP32\n');
            };
            
            ws.onmessage = function(event) {
                addToTerminal(event.data);
            };
            
            ws.onclose = function() {
                document.getElementById('connectionStatus').textContent = 'Desconectado';
                document.getElementById('connectionStatus').className = 'disconnected';
                addToTerminal('Conexión perdida. Reintentando en 3 segundos...\n');
                setTimeout(connectWebSocket, 3000);
            };
            
            ws.onerror = function() {
                addToTerminal('Error de conexión WebSocket\n');
            };
        }
        
        function addToTerminal(text) {
            const terminal = document.getElementById('terminal');
            terminal.textContent += text;
            terminal.scrollTop = terminal.scrollHeight;
        }
        
        function sendCommand() {
            const input = document.getElementById('commandInput');
            const command = input.value.trim();
            
            if (command && ws && ws.readyState === WebSocket.OPEN) {
                ws.send(command);
                commandCount++;
                document.getElementById('commandCount').textContent = commandCount;
                input.value = '';
                addToTerminal('>>> ' + command + '\n');
            }
        }
        
        function sendQuickCommand(command) {
            if (ws && ws.readyState === WebSocket.OPEN) {
                ws.send(command);
                commandCount++;
                document.getElementById('commandCount').textContent = commandCount;
                addToTerminal('>>> ' + command + '\n');
            }
        }
        
        function clearTerminal() {
            document.getElementById('terminal').textContent = '';
        }
        
        function handleEnter(event) {
            if (event.key === 'Enter') {
                sendCommand();
            }
        }
        
        // Obtener IP del ESP32
        fetch('/status')
            .then(response => response.json())
            .then(data => {
                document.getElementById('ipAddress').textContent = data.ip_address;
            })
            .catch(error => {
                document.getElementById('ipAddress').textContent = 'Error obteniendo IP';
            });
        
        // Conectar WebSocket al cargar la página
        connectWebSocket();
    </script>
</body>
</html>
)html";
}
