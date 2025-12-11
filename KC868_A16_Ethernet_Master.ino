#include <Wire.h>
#include <ETH.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HardwareSerial.h>

// ═══════════════════════════════════════════════════════════════
//          🌐 CONFIGURACIÓN DE CONEXIÓN: WiFi o Ethernet
// ═══════════════════════════════════════════════════════════════
// Cambiar este flag para seleccionar el tipo de conexión:
//   true  = Usar WiFi como conexión principal
//   false = Usar Ethernet como conexión principal
#define USE_WIFI true

// Credenciales WiFi (solo se usan si USE_WIFI = true)
const char* WIFI_SSID = "MERCUSYS_57B0";
const char* WIFI_PASSWORD = "96552333Aa";
// ═══════════════════════════════════════════════════════════════

// Configuración RS485 (PINOUT OFICIAL KC868-A16)
#define RS485_TX_PIN 13    // Pin TX para RS485 (OFICIAL: TXD=13)
#define RS485_RX_PIN 16    // Pin RX para RS485 (OFICIAL: RXD=16)  
#define RS485_DE_PIN 32    // Pin DE (Data Enable) para RS485 (Pin libre)
#define RS485_BAUD 9600    // Velocidad RS485

// Configuración entrada analógica CH4 con OpAmp (OPTIMIZADO PARA PULSOS DE 5ms @ 8V)
#define PULSE_INPUT_PIN 39    // GPIO39 - Entrada analógica CH4 (conectada a OpAmp)
#define PULSE_THRESHOLD 1800  // Umbral ADC optimizado para 8V (~1.45V en GPIO39)
#define PULSE_MIN_WIDTH 3     // Ancho mínimo 3ms (para capturar pulsos de 5ms con margen)
#define PULSE_DEBOUNCE 30     // Debounce entre pulsos: 30ms (6x el ancho del pulso)

HardwareSerial RS485(2);   // Usar Serial2 para RS485

// Configuración específica KC868-A16 basada en documentación oficial
#define ETH_CLK_MODE    ETH_CLOCK_GPIO17_OUT
#define ETH_PHY_POWER   12
#define ETH_PHY_MDC     23
#define ETH_PHY_MDIO    18
#define ETH_PHY_TYPE    ETH_PHY_LAN8720
#define ETH_PHY_ADDR    0

// Direcciones I2C para relés
#define RELAY_CHIP_1 0x24  // Relés 1-8
#define RELAY_CHIP_2 0x25  // Relés 9-16

// Configuración de red
IPAddress local_IP(192, 168, 2, 100);     // IP fija para la KC868-A16
IPAddress gateway(192, 168, 2, 1);        // Gateway de tu red
IPAddress subnet(255, 255, 255, 0);       // Máscara de subred

// Servidores
WiFiServer tcpServer(8080);  // Servidor TCP en puerto 8080
WebServer httpServer(80);    // Servidor HTTP en puerto 80
WiFiClient tcpClient;

// Variables de estado
bool relayStates[16] = {false}; // Estado de todos los relés
unsigned long relayTimers[16] = {0}; // Timers para auto-apagado (5 segundos)
unsigned long relayActivationTime[16] = {0}; // Timestamp de cuando se activó cada relé

// Variables para detección de pulsos 12V
volatile bool pulseDetected = false;        // Flag de interrupción
volatile unsigned long pulseStartTime = 0;  // Tiempo inicio del pulso
volatile unsigned long pulseEndTime = 0;    // Tiempo fin del pulso
volatile bool pulseActive = false;          // Estado actual del pulso
unsigned long lastPulseProcessed = 0;       // Última vez que se procesó un pulso
bool pulseProcessingEnabled = true;         // Habilitar/deshabilitar procesamiento

// Constantes de tiempo
#define RELAY_PROTECTION_TIME 2500          // Tiempo mínimo (2.5s) antes de permitir apagar relé activo

// ═══════════════════════════════════════════════════════════════
//                   DECLARACIONES FORWARD
// ═══════════════════════════════════════════════════════════════

void sendRS485Command(byte slaveId, byte relay, byte state, unsigned long delayMs);
void setRelay(int relayNumber, bool state);

// ═══════════════════════════════════════════════════════════════
//                   FUNCIONES DETECCIÓN DE PULSOS 12V
// ═══════════════════════════════════════════════════════════════

void IRAM_ATTR pulseInterrupt() {
    static unsigned long lastTransition = 0;
    unsigned long now = millis();
    int analogValue = analogRead(PULSE_INPUT_PIN);
    
    // Filtro anti-rebote reducido para pulsos más rápidos
    if (now - lastTransition < 1) {  // Ignorar cambios < 1ms (ruido)
        return;
    }
    
    // Detectar flanco de subida (8V presente)
    if (analogValue >= PULSE_THRESHOLD && !pulseActive) {
        pulseActive = true;
        pulseStartTime = now;
        lastTransition = now;
    }
    // Detectar flanco de bajada (8V ausente)
    else if (analogValue < PULSE_THRESHOLD && pulseActive) {
        pulseActive = false;
        pulseEndTime = now;
        lastTransition = now;
        
        unsigned long duration = pulseEndTime - pulseStartTime;
        
        // Verificar ancho mínimo (3ms para capturar pulsos de 5ms con margen)
        if (duration >= PULSE_MIN_WIDTH) {
            pulseDetected = true;
        }
    }
}

void initPulseDetection() {
    Serial.println("\n🔌 Inicializando detección de pulsos (optimizado para 5ms @ 8V)...");
    
    // ℹ️ INFORMACIÓN DEL SISTEMA
    Serial.println("╔═══════════════════════════════════════════════════════════╗");
    Serial.println("║     DETECCIÓN DE PULSOS - CONFIGURACIÓN OPTIMIZADA       ║");
    Serial.println("╚═══════════════════════════════════════════════════════════╝");
    Serial.println("📋 Especificaciones del pulso a detectar:");
    Serial.println("   ✅ Ancho del pulso: 5ms (medido en sistema real)");
    Serial.println("   ✅ Voltaje máximo: 8V");
    Serial.println("   ✅ Circuito: CH4 → Divisor + BAT54S + LM324 → GPIO39");
    Serial.println("   ✅ Voltaje en GPIO39: ~1.6-2V (8V entrada)");
    Serial.println("");
    Serial.println("🎯 Configuración de detección:");
    Serial.println("   • Umbral: 1800 ADC (1.45V en GPIO39 ≈ 5.8V entrada)");
    Serial.println("   • Ancho mínimo: 3ms (margen 60% del pulso real)");
    Serial.println("   • Muestreo: 2kHz (cada 0.5ms = 10 muestras por pulso)");
    Serial.println("   • Debounce: 30ms entre pulsos");
    Serial.println("   • Filtro ruido: < 1ms ignorado");
    
    // Configurar GPIO39 como entrada analógica
    pinMode(PULSE_INPUT_PIN, INPUT);
    
    // Configurar ADC con mejor precisión
    analogReadResolution(12);  // 12 bits = 0-4095
    analogSetAttenuation(ADC_11db);  // Atenuación 11dB
    
    // CALIBRACIÓN EXTENDIDA - 50 muestras para mayor precisión
    delay(100);
    
    Serial.println("\n🔍 Calibración del sistema (50 muestras)...");
    int readings[50];
    int minReading = 4095;
    int maxReading = 0;
    float avgReading = 0;
    
    for (int i = 0; i < 50; i++) {
        readings[i] = analogRead(PULSE_INPUT_PIN);
        avgReading += readings[i];
        if (readings[i] < minReading) minReading = readings[i];
        if (readings[i] > maxReading) maxReading = readings[i];
        delay(5);
    }
    avgReading /= 50.0;
    
    float avgVoltage = (avgReading * 3.3) / 4095.0;
    float minVoltage = (minReading * 3.3) / 4095.0;
    float maxVoltage = (maxReading * 3.3) / 4095.0;
    
    Serial.println("📊 Resultados calibración:");
    Serial.print("   • Promedio: ");
    Serial.print(avgReading, 0);
    Serial.print(" ADC (");
    Serial.print(avgVoltage, 3);
    Serial.println("V)");
    
    Serial.print("   • Rango: ");
    Serial.print(minReading);
    Serial.print("-");
    Serial.print(maxReading);
    Serial.print(" ADC (");
    Serial.print(minVoltage, 3);
    Serial.print("V - ");
    Serial.print(maxVoltage, 3);
    Serial.println("V)");
    
    Serial.print("   • Variación: ±");
    Serial.print((maxReading - minReading) / 2);
    Serial.print(" ADC (±");
    Serial.print((maxVoltage - minVoltage) / 2, 3);
    Serial.println("V)");
    
    // Calcular SNR (Signal-to-Noise Ratio)
    int noiseLevel = maxReading - minReading;
    int signalLevel = PULSE_THRESHOLD - avgReading;
    
    Serial.print("   • Nivel de ruido: ");
    Serial.print(noiseLevel);
    Serial.println(" ADC");
    
    Serial.print("   • Margen señal-ruido: ");
    Serial.print(signalLevel);
    Serial.print(" ADC (");
    Serial.print((signalLevel * 100.0) / PULSE_THRESHOLD, 1);
    Serial.println("%)");
    
    // ESTIMACIÓN DE VOLTAJE DE ENTRADA
    float estimatedInputVoltage = avgVoltage * 4.0;
    
    Serial.print("\n💡 Voltaje de entrada estimado: ~");
    Serial.print(estimatedInputVoltage, 1);
    Serial.println("V");
    
    if (estimatedInputVoltage < 1.0) {
        Serial.println("   ℹ️  Estado: SIN SEÑAL (línea en reposo)");
    } else if (estimatedInputVoltage >= 1.0 && estimatedInputVoltage <= 10.0) {
        Serial.println("   ⚠️  ADVERTENCIA: Posible señal presente en reposo");
    }
    
    // VALIDACIÓN DE SEGURIDAD
    if (avgVoltage > 3.3) {
        Serial.println("\n❌ ERROR CRÍTICO: Voltaje > 3.3V en GPIO39");
        Serial.println("❌ DESCONECTAR SEÑAL INMEDIATAMENTE");
        pulseProcessingEnabled = false;
        return;
    }
    else if (avgVoltage > 2.5) {
        Serial.println("\n⚠️  ADVERTENCIA: Voltaje cercano al límite (>2.5V)");
    }
    else if (signalLevel < 200) {
        Serial.println("\n⚠️  ADVERTENCIA: Margen señal-ruido BAJO (<200 ADC)");
        Serial.println("⚠️  Puede haber falsas detecciones o pérdida de pulsos");
    }
    else {
        Serial.println("\n✅ Voltaje en rango ÓPTIMO");
        Serial.println("✅ Margen señal-ruido EXCELENTE");
    }
    
    // Configurar timer con frecuencia optimizada para pulsos de 5ms
    // API nueva ESP32 Arduino Core 3.x - Muestreo cada 500µs = 2kHz
    hw_timer_t * timer = timerBegin(1000000);  // 1MHz (1 microsegundo de resolución)
    timerAttachInterrupt(timer, &pulseInterrupt);
    timerAlarm(timer, 500, true, 0);  // 500µs = 0.5ms = 2kHz muestreo, auto-reload
    
    Serial.println("\n✅ Sistema de detección configurado");
    Serial.print("📍 Pin: GPIO");
    Serial.println(PULSE_INPUT_PIN);
    
    Serial.print("🎯 Umbral: ");
    Serial.print(PULSE_THRESHOLD);
    Serial.print(" ADC (");
    Serial.print((PULSE_THRESHOLD * 3.3) / 4095, 2);
    Serial.print("V) → activa con ~");
    Serial.print(((PULSE_THRESHOLD * 3.3) / 4095) * 4.0, 1);
    Serial.println("V entrada");
    
    Serial.print("⏱️  Ancho mínimo: ");
    Serial.print(PULSE_MIN_WIDTH);
    Serial.print("ms (");
    Serial.print((PULSE_MIN_WIDTH * 100) / 5);  // % del pulso de 5ms
    Serial.println("% del pulso real)");
    
    Serial.print("🔄 Frecuencia muestreo: 2kHz (");
    Serial.print(5 * 2);  // 10 muestras para pulso de 5ms @ 2kHz
    Serial.println(" muestras por pulso)");
    
    Serial.print("⏳ Debounce: ");
    Serial.print(PULSE_DEBOUNCE);
    Serial.println("ms entre pulsos");
    
    Serial.println("🛡️  Protección: Divisor + BAT54S + LM324");
    
    Serial.println("\n📈 Capacidad de detección:");
    Serial.print("   • Pulsos detectables: ");
    Serial.print(1000 / (5 + PULSE_DEBOUNCE));
    Serial.println(" pulsos/seg máximo");
    
    Serial.println("   • Resolución temporal: 0.5ms");
    Serial.println("   • Precisión ancho: ±0.5ms");
    
    Serial.println("\n🚀 Sistema listo para detectar pulsos de 5ms @ 8V");
}

void processPulseDetection() {
    if (!pulseProcessingEnabled) return;
    
    // Verificar si se detectó un pulso válido
    if (pulseDetected) {
        pulseDetected = false;  // Limpiar flag
        
        // Debounce mejorado para pulsos rápidos
        unsigned long now = millis();
        if (now - lastPulseProcessed < PULSE_DEBOUNCE) {
            Serial.print("🚫 Pulso ignorado (debounce: ");
            Serial.print(now - lastPulseProcessed);
            Serial.print("ms < ");
            Serial.print(PULSE_DEBOUNCE);
            Serial.println("ms)");
            return;
        }
        lastPulseProcessed = now;
        
        // Calcular duración del pulso
        unsigned long pulseDuration = pulseEndTime - pulseStartTime;
        
        // Validar que el ancho está cerca de lo esperado (5ms ±50%)
        if (pulseDuration < 3 || pulseDuration > 10) {
            Serial.print("⚠️  Pulso anormal detectado: ");
            Serial.print(pulseDuration);
            Serial.println("ms (esperado: 5ms ±2ms)");
            // Continuar procesando de todas formas
        }
        
        Serial.println("\n╔═══════════════════════════════════════════════════╗");
        Serial.println("║     🚨 ¡PULSO DE 8V DETECTADO!                  ║");
        Serial.println("╚═══════════════════════════════════════════════════╝");
        Serial.print("📊 Duración medida: ");
        Serial.print(pulseDuration);
        Serial.print("ms (esperado: ~5ms)");
        
        if (pulseDuration >= 4 && pulseDuration <= 6) {
            Serial.println(" ✅ PERFECTO");
        } else if (pulseDuration >= 3 && pulseDuration <= 8) {
            Serial.println(" ⚠️  Aceptable");
        } else {
            Serial.println(" ❌ Fuera de rango");
        }
        
        Serial.print("⏰ Timestamp: ");
        Serial.print(now);
        Serial.println("ms");
        
        // Leer voltaje actual para log
        int currentADC = analogRead(PULSE_INPUT_PIN);
        float currentVoltage = (currentADC * 3.3) / 4095.0;
        Serial.print("📈 Voltaje actual GPIO39: ");
        Serial.print(currentADC);
        Serial.print(" ADC (");
        Serial.print(currentVoltage, 2);
        Serial.println("V)");
        
        // Ejecutar acción personalizada
        handlePulseAction(pulseDuration);
        
        Serial.println("═════════════════════════════════════════════════════\n");
    }
}

void handlePulseAction(unsigned long duration) {
    Serial.println("⚡ Ejecutando acción por pulso de 8V...");
    
    // 🎯 ACCIÓN ESPECÍFICA PARA PULSOS DE ~5ms
    // Como todos los pulsos son de 5ms, no clasificamos por duración
    
    Serial.println("📋 Pulso de 5ms detectado → Acción configurada:");
    
    // 🔴 ACCIÓN: APAGAR TODAS LAS SALIDAS (con protección de 2.5s)
    Serial.println("   🔴 Apagando relés 1-16 del MASTER...");
    
    unsigned long now = millis();
    int relaysOff = 0;
    int relaysProtected = 0;
    
    for (int i = 1; i <= 16; i++) {
        if (relayStates[i-1]) {  // Si el relé está encendido
            // Verificar si ha pasado el tiempo de protección
            unsigned long timeSinceActivation = now - relayActivationTime[i-1];
            
            if (timeSinceActivation >= RELAY_PROTECTION_TIME) {
                // Relé puede ser apagado (ya pasaron 2.5 segundos)
                setRelay(i, false);
                relaysOff++;
            } else {
                // Relé está protegido (menos de 2.5 segundos desde activación)
                unsigned long remaining = RELAY_PROTECTION_TIME - timeSinceActivation;
                Serial.print("   🛡️  Relé ");
                Serial.print(i);
                Serial.print(" PROTEGIDO (faltan ");
                Serial.print(remaining);
                Serial.println("ms para poder apagarse)");
                relaysProtected++;
            }
        }
    }
    
    // Limpiar timers solo de los relés apagados
    for (int i = 0; i < 16; i++) {
        if (!relayStates[i]) {  // Si el relé está apagado
            relayTimers[i] = 0;
        }
    }
    
    Serial.print("   ✅ ");
    Serial.print(relaysOff);
    Serial.println(" relés apagados");
    
    if (relaysProtected > 0) {
        Serial.print("   🛡️  ");
        Serial.print(relaysProtected);
        Serial.println(" relés protegidos (activados hace menos de 2.5s)");
    }
    
    if (relaysOff == 0 && relaysProtected == 0) {
        Serial.println("   ℹ️  Todos los relés ya estaban apagados");
    }
    
    // Log detallado
    Serial.println("� Detalles del pulso:");
    Serial.print("  - Inicio: ");
    Serial.print(pulseStartTime);
    Serial.println("ms");
    Serial.print("  - Fin: ");
    Serial.print(pulseEndTime);
    Serial.println("ms");
    Serial.print("  - Duración: ");
    Serial.print(duration);
    Serial.println("ms");
    
    // Contador de pulsos
    static unsigned long pulseCount = 0;
    pulseCount++;
    Serial.print("  - Total pulsos detectados: ");
    Serial.println(pulseCount);
    
    // Notificación JSON para clientes TCP si están conectados
    String notification = "{\"event\":\"pulse_detected\",";
    notification += "\"duration\":" + String(duration) + ",";
    notification += "\"timestamp\":" + String(millis()) + ",";
    notification += "\"count\":" + String(pulseCount) + ",";
    notification += "\"relays_turned_off\":" + String(relaysOff) + ",";
    notification += "\"relays_protected\":" + String(relaysProtected) + ",";
    notification += "\"action\":\"all_relays_off\"}";
    
    if (tcpClient && tcpClient.connected()) {
        tcpClient.println("PULSE: " + notification);
        Serial.println("📡 Notificación TCP enviada");
    }
    
    if (relaysProtected > 0) {
        Serial.println("✅ Acción completada - Salidas con protección de 2.5s activa");
    } else {
        Serial.println("✅ Acción completada - Todas las salidas apagadas");
    }
}

void setup() {
    Serial.begin(115200);
    delay(2000);  // Esperar estabilización
    Serial.println("=== KC868-A16 ETHERNET OPTIMIZADO ===");
    
    // Mostrar información del chip
    Serial.print("Chip: ");
    Serial.println(ESP.getChipModel());
    Serial.print("Revisión: ");
    Serial.println(ESP.getChipRevision());
    Serial.print("Frecuencia CPU: ");
    Serial.print(ESP.getCpuFreqMHz());
    Serial.println(" MHz");
    Serial.print("Memoria libre: ");
    Serial.print(ESP.getFreeHeap() / 1024);
    Serial.println(" KB");
    
    // Inicializar conexión según configuración
    if (USE_WIFI) {
        Serial.println("\n📶 MODO WIFI SELECCIONADO");
        Serial.println("🚀 Inicializando WiFi como conexión principal...");
        
        // Apagar Ethernet para WiFi
        ETH.end();
        delay(500);
        
        // Inicializar WiFi
        initWiFi();
    } else {
        Serial.println("\n⚡ MODO ETHERNET SELECCIONADO");
        Serial.println("🚀 Inicializando Ethernet con prioridad máxima...");
        
        // Liberar WiFi completamente para Ethernet
        WiFi.mode(WIFI_OFF);
        delay(1000);
        
        // Inicializar Ethernet PRIMERO
        initEthernet();
    }
    
    // Después inicializar I2C para relés
    Serial.println("\n🔧 Inicializando control de relés...");
    Wire.begin(4, 5);
    delay(100);
    
    // Inicializar RS485 Master (PINOUT OFICIAL KC868-A16)
    initRS485();
    
    // Inicializar detección de pulsos 12V en CH4/GPIO39
    initPulseDetection();
    
    // Verificar chips I2C
    bool chip1_ok = testChip(RELAY_CHIP_1);
    bool chip2_ok = testChip(RELAY_CHIP_2);
    
    if (chip1_ok && chip2_ok) {
        Serial.println("✅ Chips I2C OK - Relés listos");
        turnOffAllRelays();
    } else {
        Serial.println("⚠️  Problemas con chips I2C:");
        Serial.print("  Chip 0x24: ");
        Serial.println(chip1_ok ? "OK" : "ERROR");
        Serial.print("  Chip 0x25: ");
        Serial.println(chip2_ok ? "OK" : "ERROR");
    }
    
    // Verificar conectividad final
    bool hasConnectivity = false;
    String connectionType = "";
    IPAddress currentIP;
    
    if (USE_WIFI) {
        // Modo WiFi
        if (WiFi.status() == WL_CONNECTED) {
            hasConnectivity = true;
            connectionType = "WiFi";
            currentIP = WiFi.localIP();
            Serial.println("\n🎯 ¡WIFI CONECTADO EXITOSAMENTE!");
        }
    } else {
        // Modo Ethernet
        if (ETH.linkUp()) {
            hasConnectivity = true;
            connectionType = "Ethernet";
            currentIP = ETH.localIP();
            Serial.println("\n🎯 ¡ETHERNET COMO CONEXIÓN PRINCIPAL!");
        } else if (WiFi.status() == WL_CONNECTED) {
            hasConnectivity = true;
            connectionType = "WiFi (Respaldo)";
            currentIP = WiFi.localIP();
            Serial.println("\n📶 WiFi activado como respaldo");
        }
    }
    
    if (hasConnectivity) {
        // Inicializar servidores
        initTCPServer();
        initHTTPServer();
        
        Serial.println("\n╔══════════════════════════════════════════════╗");
        Serial.println("║           🎉 SISTEMA LISTO 🎉               ║");
        Serial.println("╚══════════════════════════════════════════════╝");
        
        Serial.print("🌐 Conectado por: ");
        Serial.print(connectionType);
        if (connectionType == "Ethernet") {
            Serial.println(" ⚡ (OBJETIVO CUMPLIDO!)");
        }
        
        Serial.print("📍 IP de control: ");
        Serial.println(currentIP);
        
        Serial.println("\n═══ APIs PARA POSTMAN ═══");
        Serial.print("🔗 POST http://");
        Serial.print(currentIP);
        Serial.println("/relay");
        Serial.println("   📝 Body: {\"relay\":5,\"state\":1}");
        
        Serial.print("🔗 GET  http://");
        Serial.print(currentIP);
        Serial.println("/status");
        
        Serial.println("\n✅ ¡LISTO PARA CONTROL REMOTO!");
        
    } else {
        Serial.println("\n⚠️  MODO SOLO SERIE");
        Serial.println("📝 Comandos disponibles:");
        Serial.println("  • SET,5,1  - Encender relé 5");
        Serial.println("  • STATUS   - Ver estado");
    }
}

bool testChip(byte address) {
    Wire.beginTransmission(address);
    return (Wire.endTransmission() == 0);
}

void turnOffAllRelays() {
    Serial.println("Apagando todos los relés...");
    writeToChip(RELAY_CHIP_1, 0xFF);  // Todos HIGH = OFF
    writeToChip(RELAY_CHIP_2, 0xFF);  // Todos HIGH = OFF
}

void writeToChip(byte address, byte value) {
    Wire.beginTransmission(address);
    Wire.write(value);
    Wire.endTransmission();
}

void setRelay(int relayNumber, bool state) {
    if (relayNumber < 1 || relayNumber > 16) {
        Serial.println("Error: Relé debe ser 1-16");
        return;
    }
    
    byte address;
    int pin;
    
    if (relayNumber <= 8) {
        address = RELAY_CHIP_1;
        pin = relayNumber - 1;  // 0-7
    } else {
        address = RELAY_CHIP_2;
        pin = relayNumber - 9;  // 0-7
    }
    
    // Leer estado actual del chip
    byte currentState = readFromChip(address);
    
    // Modificar solo el bit correspondiente
    if (state) {
        currentState &= ~(1 << pin);  // Clear bit (LOW = ON)
        // ⏰ NUEVA FUNCIONALIDAD: Auto-apagado en 5 segundos
        relayTimers[relayNumber - 1] = millis() + 5000; // 5000ms = 5 segundos
        // 🛡️ Registrar tiempo de activación para protección
        relayActivationTime[relayNumber - 1] = millis();
        Serial.print("⏰ Relé ");
        Serial.print(relayNumber);
        Serial.println(" se apagará automáticamente en 5 segundos");
    } else {
        currentState |= (1 << pin);   // Set bit (HIGH = OFF)
        // Cancelar timer si se apaga manualmente
        relayTimers[relayNumber - 1] = 0;
        // Limpiar tiempo de activación
        relayActivationTime[relayNumber - 1] = 0;
    }
    
    // Escribir nuevo estado
    writeToChip(address, currentState);
    
    // Actualizar estado en memoria
    relayStates[relayNumber - 1] = state;
    
    Serial.print("Relé ");
    Serial.print(relayNumber);
    Serial.print(": ");
    Serial.println(state ? "ON (5s auto-off)" : "OFF");
}

byte readFromChip(byte address) {
    Wire.requestFrom(address, (uint8_t)1);
    if (Wire.available()) {
        return Wire.read();
    }
    return 0xFF;  // Default: todos OFF
}

void checkAutoOffTimers() {
    // Verificar timers de auto-apagado (cada 100ms para precisión)
    static unsigned long lastCheck = 0;
    if (millis() - lastCheck < 100) return;
    lastCheck = millis();
    
    unsigned long now = millis();
    
    for (int i = 0; i < 16; i++) {
        // Si hay un timer activo y ha expirado
        if (relayTimers[i] > 0 && now >= relayTimers[i]) {
            // Apagar el relé automáticamente
            int relayNumber = i + 1;
            
            byte address;
            int pin;
            
            if (relayNumber <= 8) {
                address = RELAY_CHIP_1;
                pin = relayNumber - 1;
            } else {
                address = RELAY_CHIP_2;
                pin = relayNumber - 9;
            }
            
            // Leer estado actual y apagar el relé
            byte currentState = readFromChip(address);
            currentState |= (1 << pin);   // Set bit (HIGH = OFF)
            writeToChip(address, currentState);
            
            // Actualizar estado
            relayStates[i] = false;
            relayTimers[i] = 0;  // Limpiar timer
            
            Serial.print("⏰ AUTO-OFF: Relé ");
            Serial.print(relayNumber);
            Serial.println(" apagado automáticamente después de 5 segundos");
        }
    }
}

// ═══════════════════════════════════════════════════════════════
//                   FUNCIONES RS485 MASTER (NUEVOS PINES)
// ═══════════════════════════════════════════════════════════════

void initRS485() {
    Serial.println("\n🔌 Inicializando RS485 Master (PINOUT OFICIAL KC868-A16)...");
    
    // Configurar pin DE (Data Enable)
    pinMode(RS485_DE_PIN, OUTPUT);
    digitalWrite(RS485_DE_PIN, LOW);  // Modo recepción por defecto
    
    // Inicializar puerto serie RS485
    RS485.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
    
    Serial.println("✅ RS485 Master iniciado - PINOUT OFICIAL KC868-A16");
    Serial.print("📍 TXD: GPIO");
    Serial.print(RS485_TX_PIN);
    Serial.print(" (OFICIAL), RXD: GPIO");
    Serial.print(RS485_RX_PIN);
    Serial.print(" (OFICIAL), DE: GPIO");
    Serial.println(RS485_DE_PIN);
    Serial.print("⚡ Baud: ");
    Serial.println(RS485_BAUD);
    Serial.println("🎯 PINOUT SEGÚN DOCUMENTACIÓN KC868-A16");
    Serial.println("✅ GPIO17 LIBRE PARA ETHERNET CLOCK");
}

void sendRS485Command(byte slaveId, byte relay, byte state, unsigned long delayMs = 0) {
    // Protocolo extendido: [START][SLAVE_ID][COMMAND][RELAY][STATE][DELAY_4_BYTES][CHECKSUM][END]
    // START: 0xAA, END: 0x55, COMMAND: 0x01 (SET_RELAY_WITH_DELAY)
    
    byte command[] = {
        0xAA,                    // START byte
        slaveId,                 // ID del esclavo (1-247)
        0x01,                    // Comando SET_RELAY_WITH_DELAY
        relay,                   // Número de relé (1-16)
        state,                   // Estado (0=OFF, 1=ON)
        (byte)(delayMs >> 24),   // Delay byte más significativo
        (byte)(delayMs >> 16),   // Delay byte 2
        (byte)(delayMs >> 8),    // Delay byte 3
        (byte)(delayMs & 0xFF),  // Delay byte menos significativo
        0x00,                    // Checksum (se calculará)
        0x55                     // END byte
    };
    
    // Calcular checksum (XOR de todos los bytes excepto checksum y END)
    byte checksum = 0;
    for (int i = 0; i < 9; i++) {
        checksum ^= command[i];
    }
    command[9] = checksum;
    
    // 🔍 DEBUG: Mostrar comando que se va a enviar
    Serial.print("🔍 DEBUG Master → Esclavo ");
    Serial.print(slaveId);
    Serial.print(": ");
    for (int i = 0; i < 11; i++) {
        Serial.print("0x");
        if (command[i] < 16) Serial.print("0");
        Serial.print(command[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    
    // Enviar comando
    digitalWrite(RS485_DE_PIN, HIGH);  // Modo transmisión
    delay(1);  // Pequeña pausa para estabilizar
    
    RS485.write(command, 11);  // Ahora son 11 bytes en total
    RS485.flush();  // Esperar que termine la transmisión
    
    delay(2);  // Pausa entre transmisión y recepción
    digitalWrite(RS485_DE_PIN, LOW);   // Modo recepción
    
    Serial.print("📡 RS485 → Esclavo ");
    Serial.print(slaveId);
    Serial.print(", Relé ");
    Serial.print(relay);
    Serial.print(": ");
    Serial.print(state ? "ON" : "OFF");
    Serial.print(" (Delay: ");
    Serial.print(delayMs);
    Serial.println("ms)");
}

String sendRS485StatusRequest(byte slaveId) {
    // Protocolo: [START][SLAVE_ID][COMMAND][CHECKSUM][END]
    // COMMAND: 0x02 (GET_STATUS)
    
    byte command[] = {
        0xAA,        // START byte
        slaveId,     // ID del esclavo
        0x02,        // Comando GET_STATUS
        0x00,        // Checksum
        0x55         // END byte
    };
    
    // Calcular checksum
    byte checksum = 0xAA ^ slaveId ^ 0x02;
    command[3] = checksum;
    
    // Enviar comando
    digitalWrite(RS485_DE_PIN, HIGH);
    delay(1);
    
    RS485.write(command, 5);
    RS485.flush();
    
    delay(2);
    digitalWrite(RS485_DE_PIN, LOW);
    
    // Esperar respuesta (timeout 1 segundo)
    unsigned long timeout = millis() + 1000;
    String response = "";
    
    while (millis() < timeout) {
        if (RS485.available()) {
            char c = RS485.read();
            response += c;
            
            // Si recibimos byte de fin, terminar
            if (c == 0x55) break;
        }
        delay(1);
    }
    
    Serial.print("📡 RS485 STATUS ← Esclavo ");
    Serial.print(slaveId);
    Serial.print(": ");
    Serial.println(response.length() > 0 ? "Respuesta recibida" : "Sin respuesta");
    
    return response;
}

void initEthernet() {
    Serial.println("=== ETHERNET KC868-A16 CONFIGURACIÓN OFICIAL ===");
    
    // Configuración basada en documentación oficial de Kincony
    Serial.println("Parámetros oficiales KC868-A16:");
    Serial.println("• ETH_PHY_POWER: GPIO12");
    Serial.println("• ETH_PHY_MDC: GPIO23");  
    Serial.println("• ETH_PHY_MDIO: GPIO18");
    Serial.println("• ETH_CLK_MODE: GPIO17_OUT");
    Serial.println("• ETH_PHY_TYPE: LAN8720");
    Serial.println("• ETH_PHY_ADDR: 0");
    
    Serial.print("\n🚀 Inicializando Ethernet... ");
    
    // Configuración oficial KC868-A16
    bool success = ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_PHY_MDC, ETH_PHY_MDIO, ETH_PHY_POWER, ETH_CLK_MODE);
    
    if (success) {
        Serial.println("✅ ETH.begin() exitoso!");
        
        // Esperar enlace físico con timeout extendido
        Serial.print("⏳ Detectando enlace físico");
        int timeout = 150; // 15 segundos
        bool linkDetected = false;
        
        while (timeout > 0 && !linkDetected) {
            delay(100);
            if (ETH.linkUp()) {
                linkDetected = true;
                break;
            }
            if (timeout % 15 == 0) Serial.print(".");
            timeout--;
        }
        
        if (linkDetected) {
            Serial.println(" ✅ ¡ENLACE DETECTADO!");
            configureStaticIP();
            
            Serial.println("\n🎉 ¡ETHERNET KC868-A16 FUNCIONANDO!");
            Serial.println("✅ Configuración oficial exitosa");
            return;
        } else {
            Serial.println(" ❌ Sin enlace físico después de 15 segundos");
        }
    } else {
        Serial.println("❌ ETH.begin() falló con configuración oficial");
    }
    
    // Configuraciones alternativas rápidas
    Serial.println("\n🔄 Probando configuraciones alternativas...");
    
    // Alternativa 1: Sin power pin
    Serial.print("Alt 1 (sin power)... ");
    ETH.end();
    delay(1000);
    
    if (ETH.begin(ETH_PHY_LAN8720, 0, 23, 18, -1, ETH_CLOCK_GPIO17_OUT)) {
        Serial.println("✅ OK");
        delay(5000);
        if (ETH.linkUp()) {
            Serial.println("✅ ¡ENLACE ALTERNATIVO DETECTADO!");
            configureStaticIP();
            return;
        }
        Serial.println("❌ Sin enlace");
    } else {
        Serial.println("❌ Falló");
    }
    
    // Alternativa 2: Clock GPIO0
    Serial.print("Alt 2 (clock GPIO0)... ");
    ETH.end();
    delay(1000);
    
    if (ETH.begin(ETH_PHY_LAN8720, 0, 23, 18, 12, ETH_CLOCK_GPIO0_IN)) {
        Serial.println("✅ OK");
        delay(5000);
        if (ETH.linkUp()) {
            Serial.println("✅ ¡ENLACE ALTERNATIVO DETECTADO!");
            configureStaticIP();
            return;
        }
        Serial.println("❌ Sin enlace");
    } else {
        Serial.println("❌ Falló");
    }
    
    // Alternativa 3: PHY addr 1
    Serial.print("Alt 3 (PHY addr 1)... ");
    ETH.end();
    delay(1000);
    
    if (ETH.begin(ETH_PHY_LAN8720, 1, 23, 18, -1, ETH_CLOCK_GPIO17_OUT)) {
        Serial.println("✅ OK");
        delay(5000);
        if (ETH.linkUp()) {
            Serial.println("✅ ¡ENLACE ALTERNATIVO DETECTADO!");
            configureStaticIP();
            return;
        }
        Serial.println("❌ Sin enlace");
    } else {
        Serial.println("❌ Falló");
    }
    
    // Si todo falla, diagnóstico
    Serial.println("\n❌ ETHERNET NO DISPONIBLE");
    Serial.println("🔍 Verificaciones necesarias:");
    Serial.println("  1. ¿Cable Ethernet conectado y funcionando?");
    Serial.println("  2. ¿Alimentación 12V DC conectada?");
    Serial.println("  3. ¿Switch/router con luces de actividad?");
    Serial.println("  4. ¿LEDs del puerto Ethernet encendidos?");
    
    // Solo activar WiFi como respaldo si el flag está en false (modo Ethernet)
    if (!USE_WIFI) {
        Serial.println("\n🔄 Activando WiFi como respaldo...");
        delay(2000);
        initWiFi();
    } else {
        Serial.println("\n⚠️  Sin conexión de red disponible");
    }
}

void initWiFi() {
    if (USE_WIFI) {
        Serial.println("\n=== WIFI COMO CONEXIÓN PRINCIPAL ===");
    } else {
        Serial.println("\n=== WIFI COMO RESPALDO ===");
    }
    
    WiFi.mode(WIFI_STA);
    
    Serial.print("Conectando a WiFi: ");
    Serial.println(WIFI_SSID);
    
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(1000);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n✅ WiFi conectado!");
        
        Serial.println("\n=== WIFI CONECTADO ===");
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());
        Serial.print("RSSI: ");
        Serial.print(WiFi.RSSI());
        Serial.println(" dBm");
    } else {
        Serial.println("\n❌ Falló conexión WiFi");
        Serial.println("💡 Modificar credenciales en el código");
    }
}

void configureStaticIP() {
    Serial.println("Configurando IP estática...");
    if (!ETH.config(local_IP, gateway, subnet)) {
        Serial.println("Error configurando IP estática");
    }
    
    delay(2000);  // Tiempo para configuración
    
    Serial.println("\n=== ETHERNET CONECTADO ===");
    Serial.print("IP: ");
    Serial.println(ETH.localIP());
    Serial.print("Gateway: ");
    Serial.println(ETH.gatewayIP());
    Serial.print("MAC: ");
    Serial.println(ETH.macAddress());
    Serial.print("Velocidad: ");
    Serial.print(ETH.linkSpeed());
    Serial.println(" Mbps");
    Serial.print("Full Duplex: ");
    Serial.println(ETH.fullDuplex() ? "SI" : "NO");
}

void initTCPServer() {
    tcpServer.begin();
    Serial.println("Servidor TCP iniciado en puerto 8080");
}

void initHTTPServer() {
    // Configurar CORS
    httpServer.enableCORS(true);
    
    // POST /activate - ENDPOINT UNIFICADO Master + Esclavo
    httpServer.on("/activate", HTTP_POST, []() {
        String body = httpServer.arg("plain");
        Serial.println("POST /activate recibido: " + body);
        
        int relay = 0, state = 0, delayMs = 0;
        if (parseActivateJSON(body, relay, state, delayMs)) {
            if (relay >= 1 && relay <= 32 && (state == 0 || state == 1)) {
                
                // LÓGICA CORREGIDA: 1-16 Master, 17-32 Esclavo
                if (relay >= 1 && relay <= 16) {
                    // ===== CONTROL MASTER LOCAL (Relés 1-16) =====
                    Serial.print("🎯 MASTER LOCAL - Relé ");
                    Serial.print(relay);
                    Serial.print(": ");
                    Serial.println(state ? "ON" : "OFF");
                    
                    if (state == 1 && delayMs > 0) {
                        // Activar con delay personalizado
                        setRelayWithDelay(relay, true, delayMs);
                    } else if (state == 1 && delayMs == 0) {
                        // Activar permanente (sin auto-apagado)
                        setRelayPermanent(relay, true);
                    } else {
                        // Apagar
                        setRelayPermanent(relay, false);
                    }
                    
                    String response = "{\"status\":\"success\",\"target\":\"master\",\"relay\":" + String(relay) + 
                                     ",\"state\":" + String(state) + 
                                     ",\"delay\":" + String(delayMs) + 
                                     ",\"message\":\"Master Relé " + String(relay) + " " + 
                                     (state ? (delayMs == 0 ? "ON permanente" : "ON " + String(delayMs) + "ms") : "OFF") + "\"}";
                    
                    httpServer.send(200, "application/json", response);
                    
                } else if (relay >= 17 && relay <= 32) {
                    // ===== CONTROL ESCLAVO RS485 (Relés 17-32) =====
                    int slaveRelay = relay - 16; // Convertir: 17→1, 18→2, ..., 32→16
                    int slaveId = 1; // ID fijo del esclavo
                    
                    Serial.print("📡 ESCLAVO RS485 - Relé ");
                    Serial.print(slaveRelay);
                    Serial.print(" (relay ");
                    Serial.print(relay);
                    Serial.print("): ");
                    Serial.println(state ? "ON" : "OFF");
                    
                    // Enviar comando RS485 con delay
                    sendRS485Command(slaveId, slaveRelay, state, delayMs);
                    
                    String response = "{\"status\":\"success\",\"target\":\"slave\",\"slave_id\":" + String(slaveId) + 
                                     ",\"relay\":" + String(relay) + 
                                     ",\"slave_relay\":" + String(slaveRelay) + 
                                     ",\"state\":" + String(state) + 
                                     ",\"delay\":" + String(delayMs) + 
                                     ",\"message\":\"Esclavo Relé " + String(slaveRelay) + " " + 
                                     (state ? "ON" : "OFF") + " (delay: " + String(delayMs) + "ms)\"}";
                    
                    httpServer.send(200, "application/json", response);
                }
                
            } else {
                httpServer.send(400, "application/json", 
                               "{\"status\":\"error\",\"message\":\"relay 1-32, state 0-1\"}");
            }
        } else {
            httpServer.send(400, "application/json", 
                           "{\"status\":\"error\",\"message\":\"JSON inválido. Usar: {\\\"relay\\\":8,\\\"state\\\":1,\\\"delay\\\":8000}\"}");
        }
    });
    
    // POST /relay - Controlar relé (MANTENIDO PARA COMPATIBILIDAD)
    httpServer.on("/relay", HTTP_POST, []() {
        String body = httpServer.arg("plain");
        Serial.println("POST recibido: " + body);
        
        int relay = 0, state = 0;
        if (parseJSON(body, relay, state)) {
            if (relay >= 1 && relay <= 16 && (state == 0 || state == 1)) {
                setRelay(relay, state == 1);
                
                String response = "{\"status\":\"success\",\"relay\":" + String(relay) + 
                                 ",\"state\":" + String(state) + 
                                 ",\"message\":\"Relé " + String(relay) + " " + 
                                 (state ? "ON" : "OFF") + "\"}";
                
                httpServer.send(200, "application/json", response);
            } else {
                httpServer.send(400, "application/json", 
                               "{\"status\":\"error\",\"message\":\"Relay 1-16, state 0-1\"}");
            }
        } else {
            httpServer.send(400, "application/json", 
                           "{\"status\":\"error\",\"message\":\"JSON inválido\"}");
        }
    });
    
    // GET /status - Ver estado de todos los relés
    httpServer.on("/status", HTTP_GET, []() {
        String response = "{\"relays\":[";
        for (int i = 0; i < 16; i++) {
            if (i > 0) response += ",";
            response += "{\"relay\":" + String(i + 1) + 
                       ",\"state\":" + String(relayStates[i] ? 1 : 0) + 
                       ",\"status\":\"" + String(relayStates[i] ? "ON" : "OFF") + "\"}";
        }
        response += "],";
        
        // Agregar información de detección de pulsos
        int currentValue = analogRead(PULSE_INPUT_PIN);
        float voltage = (currentValue * 3.3) / 4095.0;
        
        response += "\"pulse_detection\":{";
        response += "\"enabled\":" + String(pulseProcessingEnabled ? "true" : "false") + ",";
        response += "\"gpio\":" + String(PULSE_INPUT_PIN) + ",";
        response += "\"current_adc\":" + String(currentValue) + ",";
        response += "\"current_voltage\":" + String(voltage, 2) + ",";
        response += "\"threshold_adc\":" + String(PULSE_THRESHOLD) + ",";
        response += "\"threshold_voltage\":" + String((PULSE_THRESHOLD * 3.3) / 4095, 2) + ",";
        response += "\"min_width_ms\":" + String(PULSE_MIN_WIDTH) + ",";
        response += "\"last_pulse_ms\":" + String(lastPulseProcessed) + ",";
        response += "\"current_state\":\"" + String(currentValue >= PULSE_THRESHOLD ? "HIGH" : "LOW") + "\"";
        response += "}}";
        
        httpServer.send(200, "application/json", response);
    });
    
    // POST /pulse - Controlar detección de pulsos
    httpServer.on("/pulse", HTTP_POST, []() {
        if (httpServer.hasArg("plain")) {
            String body = httpServer.arg("plain");
            
            // Parsear JSON para extraer comando
            String command = "";
            
            int enablePos = body.indexOf("\"enable\":");
            int testPos = body.indexOf("\"test\":");
            
            if (enablePos >= 0) {
                // Buscar valor después de "enable":
                int valueStart = body.indexOf(':', enablePos) + 1;
                String value = body.substring(valueStart);
                value.trim();
                value.replace("\"", "");
                value.replace("}", "");
                value.replace(" ", "");
                
                if (value == "true" || value == "1") {
                    pulseProcessingEnabled = true;
                    httpServer.send(200, "application/json", 
                                   "{\"status\":\"success\",\"message\":\"Pulse detection ENABLED\"}");
                } else if (value == "false" || value == "0") {
                    pulseProcessingEnabled = false;
                    httpServer.send(200, "application/json", 
                                   "{\"status\":\"success\",\"message\":\"Pulse detection DISABLED\"}");
                } else {
                    httpServer.send(400, "application/json", 
                                   "{\"status\":\"error\",\"message\":\"Invalid enable value\"}");
                }
            }
            else if (testPos >= 0) {
                // Ejecutar test de pulso
                Serial.println("🧪 Test de pulso ejecutado via HTTP");
                handlePulseAction(300);  // Simular pulso de 300ms
                httpServer.send(200, "application/json", 
                               "{\"status\":\"success\",\"message\":\"Pulse test executed (300ms)\"}");
            }
            else {
                httpServer.send(400, "application/json", 
                               "{\"status\":\"error\",\"message\":\"Invalid JSON format. Use {\\\"enable\\\":true/false} or {\\\"test\\\":true}\"}");
            }
        } else {
            httpServer.send(400, "application/json", 
                           "{\"status\":\"error\",\"message\":\"Missing JSON body\"}");
        }
    });
    
    // GET / - Página de ayuda
    httpServer.on("/", HTTP_GET, []() {
        String html = "<html><body>";
        html += "<h1>KC868-A16 Relay Controller</h1>";
        html += "<h2>🔧 Relay Control API:</h2>";
        html += "<p><b>POST /activate</b> - Unified relay control (Master + Slave)<br>";
        html += "Body: {\"relay\":25,\"state\":1,\"delay\":5000}</p>";
        html += "<p><b>POST /relay</b> - Master relay control<br>";
        html += "Body: {\"relay\":5,\"state\":1}</p>";
        html += "<p><b>GET /status</b> - Complete system status</p>";
        
        html += "<h2>🚨 Pulse Detection API (GPIO39):</h2>";
        html += "<p><b>POST /pulse</b> - Control pulse detection<br>";
        html += "Enable: {\"enable\":true}<br>";
        html += "Disable: {\"enable\":false}<br>";
        html += "Test: {\"test\":true}</p>";
        
        html += "<h2>📊 Current Status:</h2>";
        html += "<p>Pulse Detection: <b>" + String(pulseProcessingEnabled ? "ENABLED" : "DISABLED") + "</b></p>";
        
        int currentADC = analogRead(PULSE_INPUT_PIN);
        float currentVoltage = (currentADC * 3.3) / 4095.0;
        html += "<p>GPIO39 ADC: <b>" + String(currentADC) + "</b> (" + String(currentVoltage, 2) + "V)</p>";
        html += "<p>Threshold: <b>" + String(PULSE_THRESHOLD) + "</b> ADC (" + String((PULSE_THRESHOLD * 3.3) / 4095, 2) + "V)</p>";
        html += "<p>Current State: <b>" + String(currentADC >= PULSE_THRESHOLD ? "HIGH (12V)" : "LOW (0V)") + "</b></p>";
        
        html += "</body></html>";
        httpServer.send(200, "text/html", html);
    });
    
    httpServer.begin();
    Serial.println("Servidor HTTP iniciado en puerto 80");
}

bool parseJSON(String json, int &relay, int &state) {
    json.trim();
    
    int relayPos = json.indexOf("\"relay\":");
    int statePos = json.indexOf("\"state\":");
    
    if (relayPos == -1 || statePos == -1) return false;
    
    // Extraer valor de relay
    int relayStart = json.indexOf(":", relayPos) + 1;
    int relayEnd = json.indexOf(",", relayStart);
    if (relayEnd == -1) relayEnd = json.indexOf("}", relayStart);
    
    relay = json.substring(relayStart, relayEnd).toInt();
    
    // Extraer valor de state
    int stateStart = json.indexOf(":", statePos) + 1;
    int stateEnd = json.indexOf("}", stateStart);
    if (stateEnd == -1) stateEnd = json.indexOf(",", stateStart);
    
    state = json.substring(stateStart, stateEnd).toInt();
    
    return true;
}

bool parseActivateJSON(String json, int &relay, int &state, int &delayMs) {
    json.trim();
    
    int relayPos = json.indexOf("\"relay\":");
    int statePos = json.indexOf("\"state\":");
    int delayPos = json.indexOf("\"delay\":");
    
    if (relayPos == -1 || statePos == -1) return false;
    
    // Extraer relay
    int relayStart = json.indexOf(":", relayPos) + 1;
    int relayEnd = json.indexOf(",", relayStart);
    if (relayEnd == -1) relayEnd = json.indexOf("}", relayStart);
    relay = json.substring(relayStart, relayEnd).toInt();
    
    // Extraer state
    int stateStart = json.indexOf(":", statePos) + 1;
    int stateEnd = json.indexOf(",", stateStart);
    if (stateEnd == -1) stateEnd = json.indexOf("}", stateStart);
    state = json.substring(stateStart, stateEnd).toInt();
    
    // Extraer delay (en microsegundos, convertir a milisegundos)
    if (delayPos != -1) {
        int delayStart = json.indexOf(":", delayPos) + 1;
        int delayEnd = json.indexOf("}", delayStart);
        if (delayEnd == -1) delayEnd = json.indexOf(",", delayStart);
        int delayMicros = json.substring(delayStart, delayEnd).toInt();
        delayMs = delayMicros; // Convertir microsegundos a milisegundos: /1000, pero el usuario quiere microsegundos como milisegundos
        Serial.print("🕐 Delay recibido: ");
        Serial.print(delayMicros);
        Serial.print(" microsegundos → ");
        Serial.print(delayMs);
        Serial.println(" ms (aplicado)");
    } else {
        delayMs = 5000; // Default: 5 segundos si no se especifica
    }
    
    return true;
}

void setRelayWithDelay(int relayNumber, bool state, int delayMs) {
    if (relayNumber < 1 || relayNumber > 16) {
        Serial.println("Error: Relé debe ser 1-16");
        return;
    }
    
    byte address;
    int pin;
    
    if (relayNumber <= 8) {
        address = RELAY_CHIP_1;
        pin = relayNumber - 1;  // 0-7
    } else {
        address = RELAY_CHIP_2;
        pin = relayNumber - 9;  // 0-7
    }
    
    // Leer estado actual del chip
    byte currentState = readFromChip(address);
    
    // Modificar solo el bit correspondiente
    if (state) {
        currentState &= ~(1 << pin);  // Clear bit (LOW = ON)
        // Configurar timer personalizado
        relayTimers[relayNumber - 1] = millis() + delayMs;
        // 🛡️ Registrar tiempo de activación para protección
        relayActivationTime[relayNumber - 1] = millis();
        Serial.print("⏰ Relé ");
        Serial.print(relayNumber);
        Serial.print(" se apagará automáticamente en ");
        Serial.print(delayMs);
        Serial.println(" ms");
        Serial.print("🛡️ Relé ");
        Serial.print(relayNumber);
        Serial.println(" protegido por 2.5 segundos contra pulsos");
    } else {
        currentState |= (1 << pin);   // Set bit (HIGH = OFF)
        // Cancelar timer si se apaga manualmente
        relayTimers[relayNumber - 1] = 0;
        // Limpiar tiempo de activación
        relayActivationTime[relayNumber - 1] = 0;
    }
    
    // Escribir nuevo estado
    writeToChip(address, currentState);
    
    // Actualizar estado en memoria
    relayStates[relayNumber - 1] = state;
    
    Serial.print("💡 Relé ");
    Serial.print(relayNumber);
    Serial.print(": ");
    Serial.print(state ? "ON" : "OFF");
    if (state && delayMs > 0) {
        Serial.print(" (auto-off ");
        Serial.print(delayMs);
        Serial.print("ms)");
    }
    Serial.println();
}

void setRelayPermanent(int relayNumber, bool state) {
    if (relayNumber < 1 || relayNumber > 16) {
        Serial.println("Error: Relé debe ser 1-16");
        return;
    }
    
    byte address;
    int pin;
    
    if (relayNumber <= 8) {
        address = RELAY_CHIP_1;
        pin = relayNumber - 1;  // 0-7
    } else {
        address = RELAY_CHIP_2;
        pin = relayNumber - 9;  // 0-7
    }
    
    // Leer estado actual del chip
    byte currentState = readFromChip(address);
    
    // Modificar solo el bit correspondiente
    if (state) {
        currentState &= ~(1 << pin);  // Clear bit (LOW = ON)
        // NO configurar timer (permanente)
        relayTimers[relayNumber - 1] = 0;
        // 🛡️ Registrar tiempo de activación para protección
        relayActivationTime[relayNumber - 1] = millis();
        Serial.print("🔒 Relé ");
        Serial.print(relayNumber);
        Serial.println(" encendido PERMANENTE (sin auto-apagado)");
        Serial.print("🛡️ Relé ");
        Serial.print(relayNumber);
        Serial.println(" protegido por 2.5 segundos contra pulsos");
    } else {
        currentState |= (1 << pin);   // Set bit (HIGH = OFF)
        // Cancelar timer si se apaga manualmente
        relayTimers[relayNumber - 1] = 0;
        // Limpiar tiempo de activación
        relayActivationTime[relayNumber - 1] = 0;
    }
    
    // Escribir nuevo estado
    writeToChip(address, currentState);
    
    // Actualizar estado en memoria
    relayStates[relayNumber - 1] = state;
    
    Serial.print("💡 Relé ");
    Serial.print(relayNumber);
    Serial.print(": ");
    Serial.println(state ? "ON (PERMANENTE)" : "OFF");
}

bool parseRS485JSON(String json, int &slaveId, int &relay, int &state) {
    json.trim();
    
    int slavePos = json.indexOf("\"slave\":");
    int relayPos = json.indexOf("\"relay\":");
    int statePos = json.indexOf("\"state\":");
    
    if (slavePos == -1 || relayPos == -1 || statePos == -1) return false;
    
    // Extraer slaveId
    int slaveStart = json.indexOf(":", slavePos) + 1;
    int slaveEnd = json.indexOf(",", slaveStart);
    slaveId = json.substring(slaveStart, slaveEnd).toInt();
    
    // Extraer relay
    int relayStart = json.indexOf(":", relayPos) + 1;
    int relayEnd = json.indexOf(",", relayStart);
    if (relayEnd == -1) relayEnd = json.indexOf("}", relayStart);
    relay = json.substring(relayStart, relayEnd).toInt();
    
    // Extraer state
    int stateStart = json.indexOf(":", statePos) + 1;
    int stateEnd = json.indexOf("}", stateStart);
    if (stateEnd == -1) stateEnd = json.indexOf(",", stateStart);
    state = json.substring(stateStart, stateEnd).toInt();
    
    return true;
}

void processCommand(String command, WiFiClient* client = nullptr) {
    String response = "";
    
    if (command.startsWith("SET,")) {
        int firstComma = command.indexOf(',');
        int secondComma = command.indexOf(',', firstComma + 1);
        
        if (firstComma > 0 && secondComma > 0) {
            int relay = command.substring(firstComma + 1, secondComma).toInt();
            int state = command.substring(secondComma + 1).toInt();
            
            if (relay >= 1 && relay <= 16 && (state == 0 || state == 1)) {
                setRelay(relay, state == 1);
                response = "OK: Relé " + String(relay) + " " + (state ? "ON" : "OFF") + "\r\n";
            } else {
                response = "ERROR: Parámetros inválidos\r\n";
            }
        } else {
            response = "ERROR: Formato incorrecto\r\n";
        }
    }
    else if (command == "STATUS") {
        response = "Estado de relés:\r\n";
        for (int i = 0; i < 16; i++) {
            response += "Relé " + String(i + 1) + ": " + (relayStates[i] ? "ON" : "OFF") + "\r\n";
        }
        response += "\r\nDetección de pulsos: " + String(pulseProcessingEnabled ? "HABILITADA" : "DESHABILITADA") + "\r\n";
        response += "Último pulso: " + String(lastPulseProcessed) + "ms\r\n";
    }
    else if (command == "PULSE_ON") {
        pulseProcessingEnabled = true;
        response = "OK: Detección de pulsos HABILITADA\r\n";
    }
    else if (command == "PULSE_OFF") {
        pulseProcessingEnabled = false;
        response = "OK: Detección de pulsos DESHABILITADA\r\n";
    }
    else if (command == "PULSE_STATUS") {
        int currentValue = analogRead(PULSE_INPUT_PIN);
        float voltage = (currentValue * 3.3) / 4095.0;
        
        response = "Estado detección de pulsos:\r\n";
        response += "  - Habilitado: " + String(pulseProcessingEnabled ? "SÍ" : "NO") + "\r\n";
        response += "  - Pin GPIO: " + String(PULSE_INPUT_PIN) + "\r\n";
        response += "  - Valor ADC actual: " + String(currentValue) + "\r\n";
        response += "  - Voltaje actual: " + String(voltage, 2) + "V\r\n";
        response += "  - Umbral ADC: " + String(PULSE_THRESHOLD) + "\r\n";
        response += "  - Umbral voltaje: " + String((PULSE_THRESHOLD * 3.3) / 4095, 2) + "V\r\n";
        response += "  - Ancho mínimo: " + String(PULSE_MIN_WIDTH) + "ms\r\n";
        response += "  - Último pulso: " + String(lastPulseProcessed) + "ms\r\n";
        response += "  - Estado actual: " + String(currentValue >= PULSE_THRESHOLD ? "ALTO (12V)" : "BAJO (0V)") + "\r\n";
    }
    else if (command == "PULSE_TEST") {
        response = "Simulando detección de pulso de prueba...\r\n";
        handlePulseAction(250);  // Simular pulso de 250ms
        response += "Pulso de prueba ejecutado (250ms)\r\n";
    }
    else {
        response = "ERROR: Comando desconocido\r\n";
        response += "Comandos disponibles:\r\n";
        response += "  SET,<relé>,<estado> - Control de relés\r\n";
        response += "  STATUS - Estado del sistema\r\n";
        response += "  PULSE_ON - Habilitar detección de pulsos\r\n";
        response += "  PULSE_OFF - Deshabilitar detección de pulsos\r\n";
        response += "  PULSE_STATUS - Estado detallado de pulsos\r\n";
        response += "  PULSE_TEST - Probar acción de pulso\r\n";
    }
    
    if (client && response.length() > 0) {
        client->print(response);
    } else if (!client) {
        Serial.print(response);
    }
}

void loop() {
    // ⏰ VERIFICAR TIMERS DE AUTO-APAGADO (NUEVA FUNCIONALIDAD)
    checkAutoOffTimers();
    
    // 🚨 PROCESAR DETECCIÓN DE PULSOS 12V
    processPulseDetection();
    
    // Manejar servidor HTTP
    httpServer.handleClient();
    
    // Manejar conexiones TCP
    WiFiClient newClient = tcpServer.available();
    
    if (newClient) {
        Serial.print("Nueva conexión TCP desde: ");
        Serial.println(newClient.remoteIP());
        tcpClient = newClient;
    }
    
    // Procesar comandos TCP
    if (tcpClient && tcpClient.connected()) {
        if (tcpClient.available()) {
            String command = tcpClient.readStringUntil('\n');
            command.trim();
            command.toUpperCase();
            
            Serial.print("Comando TCP: ");
            Serial.println(command);
            
            processCommand(command, &tcpClient);
        }
    } else if (tcpClient) {
        tcpClient.stop();
    }
    
    // Status cada 30 segundos
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 30000) {
        if (ETH.linkUp()) {
            Serial.print("Sistema activo - Ethernet IP: ");
            Serial.println(ETH.localIP());
        } else if (WiFi.status() == WL_CONNECTED) {
            Serial.print("Sistema activo - WiFi IP: ");
            Serial.println(WiFi.localIP());
        }
        lastPrint = millis();
    }
    
    // Comandos serie
    if (Serial.available()) {
        String command = Serial.readString();
        command.trim();
        command.toUpperCase();
        
        Serial.print("Comando serie: ");
        Serial.println(command);
        
        processCommand(command);
    }
    
    delay(10);
}

/*
╔══════════════════════════════════════════════════════════════════════════════╗
║                            DOCUMENTACIÓN API                                ║
║                          KC868-A16 RELAY CONTROLLER                         ║
╚══════════════════════════════════════════════════════════════════════════════╝

🌐 CONFIGURACIÓN DE RED:
   • IP Fija: 192.168.2.100
   • Gateway: 192.168.2.1
   • Máscara: 255.255.255.0
   • Puerto HTTP: 80
   • Puerto TCP: 8080

🔧 HARDWARE CONFIGURADO:
   • ESP32 - KC868-A16 (MASTER)
   • 16 Relés locales controlados por I2C
   • Ethernet LAN8720 PHY
   • RS485 Master: TXD=GPIO13, RXD=GPIO16, DE=GPIO32 (PINOUT OFICIAL KC868-A16)
   • Direcciones I2C: 0x24 (Relés 1-8), 0x25 (Relés 9-16)
   • ⏰ AUTO-APAGADO: Relés se apagan automáticamente después de 5 segundos

═══════════════════════════════════════════════════════════════════════════════
                                 APIs HTTP REST
═══════════════════════════════════════════════════════════════════════════════

🔗 1. CONTROL UNIFICADO - MASTER + ESCLAVO (POST) ⭐ CORREGIDO ⭐
   URL: http://192.168.2.100/activate
   Método: POST
   Content-Type: application/json
   
   📝 Body JSON:
   {
     "relay": [1-32],     // 1-16: Master local, 17-32: Esclavo RS485
     "state": [0-1],      // 0 = OFF, 1 = ON
     "delay": [0-999999]  // Microsegundos activo (0 = permanente)
   }
   
   🎯 LÓGICA DE CONTROL CORREGIDA:
   • Relay 1-16:  Controla relés LOCALES del master KC868-A16
   • Relay 17-32: Controla relés del ESCLAVO RS485 (ID=1)
     - Relay 17 → Esclavo relé 1
     - Relay 18 → Esclavo relé 2
     - ...
     - Relay 32 → Esclavo relé 16
   • Delay 0:    Encendido PERMANENTE (sin auto-apagado)
   • Delay >0:   Auto-apagado después de X microsegundos
   
   📋 Ejemplos:
   • Encender relé 8 del master por 8 segundos:
     POST http://192.168.2.100/activate
     {"relay":8,"state":1,"delay":8000}
   
   • Encender relé 3 del esclavo permanente:
     POST http://192.168.2.100/activate
     {"relay":19,"state":1,"delay":0}
   
   • Apagar relé 12 del master:
     POST http://192.168.2.100/activate
     {"relay":12,"state":0,"delay":0}
   
   ✅ Respuesta exitosa (200) - Master:
   {
     "status": "success",
     "target": "master",
     "relay": 8,
     "state": 1,
     "delay": 8000,
     "message": "Master Relé 8 ON 8000ms"
   }
   
   ✅ Respuesta exitosa (200) - Esclavo:
   {
     "status": "success",
     "target": "slave",
     "slave_id": 1,
     "relay": 19,
     "slave_relay": 3,
     "state": 1,
     "delay": 0,
     "message": "Esclavo Relé 3 ON"
   }

───────────────────────────────────────────────────────────────────────────────

🔗 2. CONTROL DE RELÉ LEGACY (POST) - Mantenido para compatibilidad
   URL: http://192.168.2.100/relay
   Método: POST
   Content-Type: application/json
   
   📝 Body JSON:
   {
     "relay": [1-16],    // Número del relé (1 al 16)
     "state": [0-1]      // 0 = OFF, 1 = ON (auto-off 5s)
   }
   
   📋 Ejemplo:
     POST http://192.168.2.100/relay
     {"relay":5,"state":1}

───────────────────────────────────────────────────────────────────────────────

🔗 3. ESTADO DE TODOS LOS RELÉS (GET)
   URL: http://192.168.2.100/status
   Método: GET
   
   📋 Ejemplo:
   GET http://192.168.2.100/status
   
   ✅ Respuesta (200):
   {
     "relays": [
       {"relay": 1, "state": 0, "status": "OFF"},
       {"relay": 2, "state": 1, "status": "ON"},
       {"relay": 3, "state": 0, "status": "OFF"},
       ...
       {"relay": 16, "state": 1, "status": "ON"}
     ]
   }

───────────────────────────────────────────────────────────────────────────────

🔗 4. PÁGINA DE AYUDA (GET)
   URL: http://192.168.2.100/
   Método: GET
   
   📋 Ejemplo:
   GET http://192.168.2.100/
   
   ✅ Respuesta: Página HTML con documentación básica

═══════════════════════════════════════════════════════════════════════════════
                               CONTROL TCP (RAW)
═══════════════════════════════════════════════════════════════════════════════

🔌 CONEXIÓN TCP:
   Host: 192.168.2.100
   Puerto: 8080
   
📝 COMANDOS (terminados con \n):
   
   • SET,[relay],[state]
     - relay: 1-16
     - state: 0 (OFF) o 1 (ON)
     
   • STATUS
     - Muestra estado de todos los relés
   
📋 EJEMPLOS TCP:
   Conectar: telnet 192.168.2.100 8080
   
   Comando: SET,7,1
   Respuesta: OK: Relé 7 ON
   
   Comando: SET,7,0  
   Respuesta: OK: Relé 7 OFF
   
   Comando: STATUS
   Respuesta: 
   Estado de relés:
   Relé 1: OFF
   Relé 2: ON
   ...
   Relé 16: OFF

═══════════════════════════════════════════════════════════════════════════════
                            CONTROL POR MONITOR SERIE
═══════════════════════════════════════════════════════════════════════════════

📟 PUERTO SERIE:
   Baudios: 115200
   
📝 COMANDOS:
   Los mismos comandos TCP funcionan por serie:
   • SET,5,1    - Encender relé 5
   • SET,5,0    - Apagar relé 5  
   • STATUS     - Ver estado de todos

═══════════════════════════════════════════════════════════════════════════════
                              EJEMPLOS POSTMAN
═══════════════════════════════════════════════════════════════════════════════

🚀 COLECCIÓN POSTMAN - KC868-A16 CONTROL UNIFICADO CORREGIDO:

┌─ Activar Relé Master (1-16) ───────────────────┐
│ POST http://192.168.2.100/activate            │
│ Headers:                                       │
│   Content-Type: application/json              │
│ Body (raw):                                    │
│   {"relay":8,"state":1,"delay":8000}          │
│ (Encender relé 8 master por 8 segundos)       │
└────────────────────────────────────────────────┘

┌─ Activar Relé Esclavo (17-32) ─────────────────┐
│ POST http://192.168.2.100/activate            │
│ Headers:                                       │
│   Content-Type: application/json              │
│ Body (raw):                                    │
│   {"relay":20,"state":1,"delay":0}            │
│ (Encender relé 4 esclavo permanente)          │
└────────────────────────────────────────────────┘

┌─ Activar Último Relé Esclavo ──────────────────┐
│ POST http://192.168.2.100/activate            │
│ Headers:                                       │
│   Content-Type: application/json              │
│ Body (raw):                                    │
│   {"relay":32,"state":1,"delay":5000}         │
│ (Encender relé 16 esclavo por 5 segundos)     │
└────────────────────────────────────────────────┘

┌─ Desactivar Cualquier Relé ────────────────────┐
│ POST http://192.168.2.100/activate            │
│ Headers:                                       │
│   Content-Type: application/json              │
│ Body (raw):                                    │
│   {"relay":25,"state":0,"delay":0}            │
│ (Apagar relé 9 esclavo)                       │
└────────────────────────────────────────────────┘

┌─ Estado Todos los Relés ───────────────────────┐
│ GET http://192.168.2.100/status               │
│ (Sin headers ni body)                          │
└────────────────────────────────────────────────┘

═══════════════════════════════════════════════════════════════════════════════
                                CÓDIGOS DE ERROR
═══════════════════════════════════════════════════════════════════════════════

HTTP Status Codes:
• 200 OK - Operación exitosa
• 400 Bad Request - Parámetros inválidos
• 404 Not Found - Endpoint no encontrado
• 500 Internal Server Error - Error del sistema

Errores comunes:
• "JSON inválido" - Formato JSON incorrecto
• "Relay 1-16, state 0-1" - Parámetros fuera de rango
• Sin respuesta - Verificar conectividad de red

═══════════════════════════════════════════════════════════════════════════════
                              DIAGNÓSTICO DE RED
═══════════════════════════════════════════════════════════════════════════════

🔍 VERIFICACIONES:
1. Ping a la IP: ping 192.168.2.100
2. Verificar puerto HTTP: telnet 192.168.2.100 80  
3. Verificar puerto TCP: telnet 192.168.2.100 8080
4. Monitor serie para logs del sistema

🚨 SOLUCIÓN DE PROBLEMAS:
• Sin conectividad: Verificar cable Ethernet y alimentación 12V
• IP incorrecta: Verificar configuración de red (Gateway: 192.168.2.1)
• Relés no responden: Verificar conexiones I2C (SDA=4, SCL=5)

═══════════════════════════════════════════════════════════════════════════════
                                ESPECIFICACIONES
═══════════════════════════════════════════════════════════════════════════════

📋 HARDWARE REQUERIDO:
• KC868-A16 con ESP32
• Alimentación 12V DC
• Cable Ethernet CAT5/CAT6
• Switch/Router con puertos libres

⚡ CARACTERÍSTICAS:
• 16 relés independientes (5A/250VAC, 5A/30VDC)
• 🎯 CONTROL UNIFICADO: 1-7 Master, 8-16 Esclavo RS485 via /activate
• Delay personalizable por relé (0 = permanente, >0 = auto-apagado)
• Control simultáneo HTTP + TCP + Serie + RS485
• Master RS485 para controlar múltiples esclavos
• IP fija configurable
• CORS habilitado para APIs web
• Auto-reconexión de red
• Logs detallados por monitor serie
• 📡 PROTOCOLO RS485: Comunicación robusta con checksum

🔧 CONFIGURACIÓN I2C:
• SDA: GPIO 4
• SCL: GPIO 5  
• Chip 1 (0x24): Relés 1-8
• Chip 2 (0x25): Relés 9-16
• Lógica invertida: LOW=ON, HIGH=OFF

🌐 CONFIGURACIÓN ETHERNET:
• PHY: LAN8720
• MDC: GPIO 23
• MDIO: GPIO 18  
• Power: GPIO 12
• Clock: GPIO 17 (Output) ⚡ EXCLUSIVO PARA ETHERNET

📡 CONFIGURACIÓN RS485 MASTER (PINOUT OFICIAL KC868-A16):
• TXD: GPIO 13 (Oficial según documentación)
• RXD: GPIO 16 (Oficial según documentación)
• DE (Data Enable): GPIO 32 (Pin libre)
• Velocidad: 9600 baud
• Protocolo: [START][SLAVE_ID][CMD][DATA][CHECKSUM][END]
• Soporte para 247 esclavos simultáneos

� DETECCIÓN DE PULSOS 12V (ENTRADA ANALÓGICA CH4):
• Pin: GPIO 39 (Entrada analógica CH4)
• Conexión: OpAmp → GPIO39
• Resolución ADC: 12 bits (0-4095)
• Umbral: 2048 ADC (~1.65V después del OpAmp)
• Ancho mínimo: 10ms (filtro anti-ruido)
• Muestreo: 1kHz (cada 1ms por timer)
• Acciones configurables por duración de pulso

�🔌 CONEXIÓN RS485:
• A+ (Data+) - Conectar a todos los esclavos
• B- (Data-) - Conectar a todos los esclavos  
• GND común entre master y esclavos
• Terminaciones de línea: 120Ω en extremos del bus
• Distancia máxima: 1200m
• Velocidad máxima: 10 Mbps (configurado: 9600 baud)

🎯 RESOLUCIÓN DE CONFLICTOS:
• GPIO17: EXCLUSIVO para Ethernet Clock (ya no usado por RS485)
• GPIO4: EXCLUSIVO para I2C SDA (ya no usado por RS485)  
• GPIO39: EXCLUSIVO para detección de pulsos 12V (entrada analógica)
• Pines RS485 oficiales: TXD=GPIO13, RXD=GPIO16, DE=GPIO32
• PINOUT CONFIRMADO POR DOCUMENTACIÓN OFICIAL KC868-A16

═══════════════════════════════════════════════════════════════════════════════
Versión: KC868-A16 Unified Controller v5.0 - ¡DETECCIÓN DE PULSOS 12V!
Fecha: Octubre 2025
Autor: Sistema de Control Industrial
Características: Master(1-16) + Esclavo RS485(17-32) + Detección Pulsos 12V + Delay personalizable
Nuevas funciones: Detección de pulsos 12V en GPIO39 con OpAmp, acciones configurables
═══════════════════════════════════════════════════════════════════════════════
*/