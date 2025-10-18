#include <Wire.h>
#include <HardwareSerial.h>

// Configuración RS485 Esclavo (PINOUT OFICIAL KC868-A16)
#define RS485_TX_PIN 13    // Pin TX para RS485 (OFICIAL: TXD=13)
#define RS485_RX_PIN 16    // Pin RX para RS485 (OFICIAL: RXD=16)  
#define RS485_DE_PIN 32    // Pin DE (Data Enable) para RS485 (Pin libre)
#define RS485_BAUD 9600    // Velocidad RS485

// ID único del esclavo (CAMBIAR PARA CADA DISPOSITIVO)
#define SLAVE_ID 1         // ID de este esclavo (1-247)

HardwareSerial RS485(2);   // Usar Serial2 para RS485

// Direcciones I2C para relés
#define RELAY_CHIP_1 0x24  // Relés 1-8
#define RELAY_CHIP_2 0x25  // Relés 9-16

// Variables de estado
bool relayStates[16] = {false}; // Estado de todos los relés
unsigned long relayTimers[16] = {0}; // Timers para auto-apagado (5 segundos)

// Buffer para comandos RS485 (aumentado para comando con delay)
byte commandBuffer[15];  // Aumentado para protocolo extendido
int bufferIndex = 0;

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("=== KC868-A16 RS485 ESCLAVO (PINOUT OFICIAL) ===");
    Serial.print("ID Esclavo: ");
    Serial.println(SLAVE_ID);
    Serial.println("🔗 Pines RS485 OFICIALES: RXD=16, TXD=13");
    
    // Inicializar I2C para relés
    Serial.println("\n🔧 Inicializando control de relés...");
    Wire.begin(4, 5);
    delay(100);
    
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
    
    // Inicializar RS485
    initRS485();
    
    Serial.println("\n╔══════════════════════════════════════════════╗");
    Serial.println("║        🎉 ESCLAVO RS485 LISTO 🎉            ║");
    Serial.println("╚══════════════════════════════════════════════╝");
    Serial.println("📡 Esperando comandos del master...");
}

void initRS485() {
    Serial.println("\n🔌 Inicializando RS485 Esclavo (PINOUT OFICIAL KC868-A16)...");
    
    // Configurar pin DE (Data Enable)
    pinMode(RS485_DE_PIN, OUTPUT);
    digitalWrite(RS485_DE_PIN, LOW);  // Modo recepción por defecto
    
    // Inicializar puerto serie RS485
    RS485.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
    
    Serial.println("✅ RS485 Esclavo iniciado - PINOUT OFICIAL");
    Serial.print("📍 TXD: GPIO");
    Serial.print(RS485_TX_PIN);
    Serial.print(" (OFICIAL), RXD: GPIO");
    Serial.print(RS485_RX_PIN);
    Serial.print(" (OFICIAL), DE: GPIO");
    Serial.println(RS485_DE_PIN);
    Serial.print("⚡ Baud: ");
    Serial.println(RS485_BAUD);
    Serial.println("🎯 PINOUT SEGÚN DOCUMENTACIÓN KC868-A16");
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
    Serial.print("🔧 I2C WRITE → 0x");
    Serial.print(address, HEX);
    Serial.print(" = 0x");
    Serial.print(value, HEX);
    
    Wire.beginTransmission(address);
    Wire.write(value);
    byte result = Wire.endTransmission();
    
    Serial.print(" (resultado: ");
    switch(result) {
        case 0: Serial.println("✅ OK)"); break;
        case 1: Serial.println("❌ Datos muy largos)"); break;
        case 2: Serial.println("❌ NACK en dirección)"); break;
        case 3: Serial.println("❌ NACK en datos)"); break;
        case 4: Serial.println("❌ Error desconocido)"); break;
        default: Serial.print("❌ Error "); Serial.print(result); Serial.println(")"); break;
    }
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
        // ⏰ Auto-apagado en 5 segundos
        relayTimers[relayNumber - 1] = millis() + 5000;
        Serial.print("⏰ Relé ");
        Serial.print(relayNumber);
        Serial.println(" se apagará automáticamente en 5 segundos");
    } else {
        currentState |= (1 << pin);   // Set bit (HIGH = OFF)
        // Cancelar timer si se apaga manualmente
        relayTimers[relayNumber - 1] = 0;
    }
    
    // Escribir nuevo estado
    writeToChip(address, currentState);
    
    // Actualizar estado en memoria
    relayStates[relayNumber - 1] = state;
    
    Serial.print("💡 Relé ");
    Serial.print(relayNumber);
    Serial.print(": ");
    Serial.println(state ? "ON (5s auto-off)" : "OFF");
}

void setRelayWithDelay(int relayNumber, bool state, unsigned long delayMs) {
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
    
    Serial.print("🔧 DEBUG I2C - Dirección: 0x");
    Serial.print(address, HEX);
    Serial.print(", Pin: ");
    Serial.print(pin);
    Serial.print(", Estado actual: 0x");
    Serial.println(currentState, HEX);
    
    // Modificar solo el bit correspondiente
    if (state) {
        currentState &= ~(1 << pin);  // Clear bit (LOW = ON)
        
        // Configurar timer según delay recibido
        if (delayMs > 0) {
            relayTimers[relayNumber - 1] = millis() + delayMs;
            Serial.print("⏰ Relé ");
            Serial.print(relayNumber);
            Serial.print(" se apagará automáticamente en ");
            Serial.print(delayMs);
            Serial.println(" ms");
        } else {
            // Delay 0 = permanente
            relayTimers[relayNumber - 1] = 0;
            Serial.print("🔒 Relé ");
            Serial.print(relayNumber);
            Serial.println(" encendido permanentemente");
        }
    } else {
        currentState |= (1 << pin);   // Set bit (HIGH = OFF)
        // Cancelar timer si se apaga manualmente
        relayTimers[relayNumber - 1] = 0;
    }
    
    Serial.print("🔧 DEBUG I2C - Nuevo estado: 0x");
    Serial.print(currentState, HEX);
    Serial.print(" (bit ");
    Serial.print(pin);
    Serial.print(": ");
    Serial.print(state ? "LOW=ON" : "HIGH=OFF");
    Serial.println(")");
    
    // Escribir nuevo estado
    writeToChip(address, currentState);
    
    // Verificar escritura
    byte verifyState = readFromChip(address);
    Serial.print("🔧 DEBUG I2C - Estado verificado: 0x");
    Serial.print(verifyState, HEX);
    if (verifyState == currentState) {
        Serial.println(" ✅ Escritura exitosa");
    } else {
        Serial.println(" ❌ Error de escritura I2C");
    }
    
    // Actualizar estado en memoria
    relayStates[relayNumber - 1] = state;
    
    Serial.print("💡 Relé ");
    Serial.print(relayNumber);
    Serial.print(": ");
    if (state) {
        if (delayMs > 0) {
            Serial.print("ON (");
            Serial.print(delayMs);
            Serial.println("ms auto-off)");
        } else {
            Serial.println("ON (permanente)");
        }
    } else {
        Serial.println("OFF");
    }
}

byte readFromChip(byte address) {
    Serial.print("🔧 I2C READ ← 0x");
    Serial.print(address, HEX);
    
    Wire.requestFrom(address, (uint8_t)1);
    if (Wire.available()) {
        byte value = Wire.read();
        Serial.print(" = 0x");
        Serial.print(value, HEX);
        Serial.println(" ✅");
        return value;
    } else {
        Serial.println(" = 0xFF ❌ Sin respuesta");
        return 0xFF;  // Default: todos OFF
    }
}

void checkAutoOffTimers() {
    // Verificar timers de auto-apagado
    static unsigned long lastCheck = 0;
    if (millis() - lastCheck < 100) return;
    lastCheck = millis();
    
    unsigned long now = millis();
    
    for (int i = 0; i < 16; i++) {
        if (relayTimers[i] > 0 && now >= relayTimers[i]) {
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
            
            byte currentState = readFromChip(address);
            currentState |= (1 << pin);   // Set bit (HIGH = OFF)
            writeToChip(address, currentState);
            
            relayStates[i] = false;
            relayTimers[i] = 0;
            
            Serial.print("⏰ AUTO-OFF: Relé ");
            Serial.print(relayNumber);
            Serial.println(" apagado automáticamente");
        }
    }
}

void processRS485Command() {
    if (!RS485.available()) return;
    
    while (RS485.available()) {
        byte b = RS485.read();
        
        // Detectar inicio de comando
        if (b == 0xAA && bufferIndex == 0) {
            commandBuffer[bufferIndex++] = b;
        }
        // Continuar llenando buffer
        else if (bufferIndex > 0) {
            commandBuffer[bufferIndex++] = b;
            
            // Comando completo detectado (ahora 11 bytes para protocolo extendido)
            if (b == 0x55 || bufferIndex >= 15) {
                processCommand();
                bufferIndex = 0;
                break;
            }
        }
    }
}

void processCommand() {
    // Compatibilidad: Soporte para protocolo antiguo (7 bytes) y nuevo (11 bytes)
    Serial.print("🔍 DEBUG: Comando recibido - ");
    Serial.print(bufferIndex);
    Serial.print(" bytes: ");
    for (int i = 0; i < bufferIndex && i < 15; i++) {
        Serial.print("0x");
        if (commandBuffer[i] < 16) Serial.print("0");
        Serial.print(commandBuffer[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    
    if (bufferIndex < 7) {
        Serial.print("❌ Comando RS485 muy corto: ");
        Serial.print(bufferIndex);
        Serial.println(" bytes (mínimo: 7)");
        return;
    }
    
    // Verificar que es para este esclavo
    byte targetSlave = commandBuffer[1];
    if (targetSlave != SLAVE_ID) {
        Serial.print("🚫 Comando no es para este esclavo (ID: ");
        Serial.print(targetSlave);
        Serial.print(", esperado: ");
        Serial.print(SLAVE_ID);
        Serial.println(")");
        return; // No es para nosotros, ignorar silenciosamente
    }
    
    byte command = commandBuffer[2];
    
    // Comando SET_RELAY (soporte para ambos protocolos)
    if (command == 0x01) {
        byte relay = commandBuffer[3];
        byte state = commandBuffer[4];
        unsigned long delayMs = 0;  // Default: sin delay (compatibilidad)
        
        // Protocolo NUEVO (11 bytes) - incluye delay
        if (bufferIndex >= 11) {
            Serial.println("🔄 Detectado protocolo NUEVO (11+ bytes)");
            
            // Verificar checksum para protocolo nuevo (9 bytes)
            byte checksum = 0;
            for (int i = 0; i < 9; i++) {
                checksum ^= commandBuffer[i];
            }
            
            Serial.print("🔐 Checksum calculado: 0x");
            Serial.print(checksum, HEX);
            Serial.print(", recibido: 0x");
            Serial.println(commandBuffer[9], HEX);
            
            if (checksum != commandBuffer[9]) {
                Serial.println("❌ Checksum RS485 inválido (protocolo nuevo)");
                return;
            }
            
            // Reconstruir delay de 32 bits desde 4 bytes
            delayMs |= ((unsigned long)commandBuffer[5] << 24);
            delayMs |= ((unsigned long)commandBuffer[6] << 16);
            delayMs |= ((unsigned long)commandBuffer[7] << 8);
            delayMs |= ((unsigned long)commandBuffer[8]);
            
            Serial.print("📡 RS485 ← Master (NUEVO): Relé ");
            Serial.print(relay);
            Serial.print(" → ");
            Serial.print(state ? "ON" : "OFF");
            Serial.print(" (Delay: ");
            Serial.print(delayMs);
            Serial.println("ms)");
            
        } 
        // Protocolo ANTIGUO (7 bytes) - sin delay
        else if (bufferIndex >= 7) {
            Serial.println("🔄 Detectado protocolo ANTIGUO (7-10 bytes)");
            
            // Verificar checksum para protocolo antiguo (5 bytes)
            byte checksum = 0;
            for (int i = 0; i < 5; i++) {
                checksum ^= commandBuffer[i];
            }
            
            Serial.print("🔐 Checksum calculado: 0x");
            Serial.print(checksum, HEX);
            Serial.print(", recibido: 0x");
            Serial.println(commandBuffer[5], HEX);
            
            if (checksum != commandBuffer[5]) {
                Serial.println("❌ Checksum RS485 inválido (protocolo antiguo)");
                return;
            }
            
            delayMs = 5000;  // Default: 5 segundos (comportamiento original)
            
            Serial.print("📡 RS485 ← Master (ANTIGUO): Relé ");
            Serial.print(relay);
            Serial.print(" → ");
            Serial.print(state ? "ON" : "OFF");
            Serial.println(" (5s auto-off)");
        }
        
        Serial.print("⚙️  Ejecutando setRelayWithDelay(");
        Serial.print(relay);
        Serial.print(", ");
        Serial.print(state);
        Serial.print(", ");
        Serial.print(delayMs);
        Serial.println(")");
        
        if (relay >= 1 && relay <= 16 && (state == 0 || state == 1)) {
            setRelayWithDelay(relay, state == 1, delayMs);
            sendRS485Response(0x01, 1); // ACK
        } else {
            Serial.println("❌ Parámetros inválidos");
            sendRS485Response(0x01, 0); // NACK
        }
    }
    // Comando GET_STATUS
    else if (command == 0x02) {
        Serial.println("📡 RS485 ← Master: Solicitud de estado");
        sendStatusResponse();
    }
}

void sendRS485Response(byte command, byte status) {
    // Protocolo respuesta: [START][SLAVE_ID][COMMAND][STATUS][CHECKSUM][END]
    byte response[] = {
        0xAA,        // START
        SLAVE_ID,    // ID del esclavo
        command,     // Comando original
        status,      // 1=ACK, 0=NACK
        0x00,        // Checksum
        0x55         // END
    };
    
    // Calcular checksum
    byte checksum = 0;
    for (int i = 0; i < 4; i++) {
        checksum ^= response[i];
    }
    response[4] = checksum;
    
    // Enviar respuesta
    digitalWrite(RS485_DE_PIN, HIGH);
    delay(1);
    
    RS485.write(response, 6);
    RS485.flush();
    
    delay(2);
    digitalWrite(RS485_DE_PIN, LOW);
    
    Serial.print("📡 RS485 → Master: ");
    Serial.println(status ? "ACK" : "NACK");
}

void sendStatusResponse() {
    // Crear respuesta con estado de todos los relés
    // Protocolo: [START][SLAVE_ID][0x02][16 bytes estado][CHECKSUM][END]
    byte response[21];
    response[0] = 0xAA;      // START
    response[1] = SLAVE_ID;  // ID del esclavo
    response[2] = 0x02;      // Comando GET_STATUS
    
    // 16 bytes con estado de relés
    for (int i = 0; i < 16; i++) {
        response[3 + i] = relayStates[i] ? 1 : 0;
    }
    
    // Calcular checksum
    byte checksum = 0;
    for (int i = 0; i < 19; i++) {
        checksum ^= response[i];
    }
    response[19] = checksum;
    response[20] = 0x55;     // END
    
    // Enviar respuesta
    digitalWrite(RS485_DE_PIN, HIGH);
    delay(1);
    
    RS485.write(response, 21);
    RS485.flush();
    
    delay(2);
    digitalWrite(RS485_DE_PIN, LOW);
    
    Serial.println("📡 RS485 → Master: Estado enviado");
}

void loop() {
    // Verificar timers de auto-apagado
    checkAutoOffTimers();
    
    // Procesar comandos RS485
    processRS485Command();
    
    // Status cada 30 segundos
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 30000) {
        Serial.print("💡 Esclavo ");
        Serial.print(SLAVE_ID);
        Serial.println(" activo - Esperando comandos RS485");
        lastPrint = millis();
    }
    
    // Comandos serie locales
    if (Serial.available()) {
        String command = Serial.readString();
        command.trim();
        command.toUpperCase();
        
        if (command.startsWith("SET,")) {
            int firstComma = command.indexOf(',');
            int secondComma = command.indexOf(',', firstComma + 1);
            
            if (firstComma > 0 && secondComma > 0) {
                int relay = command.substring(firstComma + 1, secondComma).toInt();
                int state = command.substring(secondComma + 1).toInt();
                
                if (relay >= 1 && relay <= 16 && (state == 0 || state == 1)) {
                    setRelay(relay, state == 1);
                } else {
                    Serial.println("Error: Parámetros inválidos");
                }
            }
        }
        else if (command == "STATUS") {
            Serial.println("Estado de relés:");
            for (int i = 0; i < 16; i++) {
                Serial.print("Relé ");
                Serial.print(i + 1);
                Serial.print(": ");
                Serial.println(relayStates[i] ? "ON" : "OFF");
            }
        }
        else if (command == "TEST") {
            Serial.println("🧪 TEST I2C - Verificando chips...");
            
            // Test chip 0x24 (relés 1-8)
            Serial.println("📋 Chip 0x24 (relés 1-8):");
            for (int i = 0; i < 8; i++) {
                Serial.print("  Relé ");
                Serial.print(i + 1);
                Serial.print(": ");
                setRelayWithDelay(i + 1, true, 1000);  // ON por 1 segundo
                delay(500);
            }
            
            delay(2000);
            
            // Test chip 0x25 (relés 9-16)
            Serial.println("📋 Chip 0x25 (relés 9-16):");
            for (int i = 8; i < 16; i++) {
                Serial.print("  Relé ");
                Serial.print(i + 1);
                Serial.print(": ");
                setRelayWithDelay(i + 1, true, 1000);  // ON por 1 segundo
                delay(500);
            }
            
            Serial.println("🧪 Test completado");
        }
    }
    
    delay(10);
}

/*
╔══════════════════════════════════════════════════════════════════════════════╗
║                          KC868-A16 RS485 ESCLAVO                            ║
║                           DOCUMENTACIÓN TÉCNICA                             ║
╚══════════════════════════════════════════════════════════════════════════════╝

🔧 CONFIGURACIÓN HARDWARE:
   • ESP32 - KC868-A16
   • 16 Relés controlados por I2C (Auto-apagado 5 segundos)
   • RS485: TXD=GPIO13, RXD=GPIO16, DE=GPIO32 (PINOUT OFICIAL)
   • Velocidad: 9600 baud
   • ID Esclavo: Configurable (1-247)

📡 PROTOCOLO RS485:

   🔗 SET RELAY (Master → Esclavo):
   [0xAA][SLAVE_ID][0x01][RELAY][STATE][CHECKSUM][0x55]
   
   📤 Respuesta ACK/NACK (Esclavo → Master):
   [0xAA][SLAVE_ID][0x01][STATUS][CHECKSUM][0x55]
   
   🔗 GET STATUS (Master → Esclavo):
   [0xAA][SLAVE_ID][0x02][CHECKSUM][0x55]
   
   📤 Respuesta Estado (Esclavo → Master):
   [0xAA][SLAVE_ID][0x02][16 bytes estado][CHECKSUM][0x55]

🔧 CONFIGURACIÓN:
   1. Cambiar SLAVE_ID para cada dispositivo (línea 11)
   2. Conectar RS485: A+, B-, GND común
   3. Terminaciones de línea si es necesario
   4. Alimentación 12V independiente para cada esclavo

⚡ CARACTERÍSTICAS:
   • Control local por monitor serie
   • Auto-apagado de relés en 5 segundos
   • Protocolo robusto con checksum
   • Respuestas automáticas al master
   • Logs detallados de comunicación

═══════════════════════════════════════════════════════════════════════════════
Versión: KC868-A16 RS485 Slave v1.0
Fecha: Octubre 2025
═══════════════════════════════════════════════════════════════════════════════
*/