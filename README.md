# KC868-A16 Controller - Sistema de Control Unificado

![KC868-A16](https://img.shields.io/badge/Hardware-KC868--A16-blue.svg)
![ESP32](https://img.shields.io/badge/Platform-ESP32-green.svg)
![Ethernet](https://img.shields.io/badge/Network-Ethernet-orange.svg)
![RS485](https://img.shields.io/badge/Protocol-RS485-red.svg)

## 📋 Descripción

Sistema de control industrial basado en KC868-A16 ESP32 que permite el control unificado de relés tanto locales (master) como remotos (esclavo) a través de Ethernet y comunicación RS485. Soporta activación temporal con delay personalizable y control permanente.

## 🏗️ Arquitectura del Sistema

```
┌─────────────────┐    Ethernet     ┌─────────────────┐
│   Cliente Web   │◄────────────────►│  KC868-A16      │
│   (Postman/API) │                  │  MASTER         │
└─────────────────┘                  │                 │
                                     │ • Relés 1-16    │
                                     │ • HTTP Server   │
                                     │ • I2C Control   │
                                     └─────────┬───────┘
                                               │ RS485
                                               ▼
                                     ┌─────────────────┐
                                     │  KC868-A16      │
                                     │  ESCLAVO        │
                                     │                 │
                                     │ • Relés 17-32   │
                                     │ • RS485 Slave   │
                                     │ • I2C Control   │
                                     └─────────────────┘
```

## 🔌 Configuración de Hardware

### KC868-A16 Master (Principal)

#### Ethernet LAN8720 PHY:
- **GPIO17**: ETH_CLK
- **GPIO12**: ETH_POWER  
- **GPIO23**: ETH_MDC
- **GPIO18**: ETH_MDIO

#### RS485 (Pinout Oficial):
- **GPIO13**: TXD (A)
- **GPIO16**: RXD (B)
- **GPIO32**: DE (Driver Enable)

#### I2C Relés:
- **GPIO4**: SDA
- **GPIO5**: SCL
- **0x24**: Dirección relés 1-8
- **0x25**: Dirección relés 9-16

### KC868-A16 Esclavo

#### RS485 (Mismo pinout):
- **GPIO13**: TXD (A)
- **GPIO16**: RXD (B)  
- **GPIO32**: DE (Driver Enable)

#### I2C Relés:
- **GPIO4**: SDA
- **GPIO5**: SCL
- **0x24**: Dirección relés 1-8
- **0x25**: Dirección relés 9-16

### Conexión RS485

```
Master KC868-A16          Esclavo KC868-A16
    A (GPIO13) ────────────── A (GPIO13)
    B (GPIO16) ────────────── B (GPIO16)
       GND     ────────────── GND
```

## 🌐 Configuración de Red

### Master KC868-A16:
- **IP Estática**: `192.168.2.100`
- **Gateway**: `192.168.2.1`
- **Subnet**: `255.255.255.0`
- **DNS**: `8.8.8.8`
- **Puerto HTTP**: `80`
- **Puerto TCP**: `8080`

## 📁 Estructura del Proyecto

```
KC868-A16-Controller/
├── README.md                                    # Este archivo
├── KC868_A16_RS485_Slave.ino                   # Código esclavo
└── KC868_A16_Ethernet_Fixed/
    ├── KC868_A16_Ethernet_Fixed.ino            # Código master principal
    └── KC868_A16_Ethernet_Fixed - copia.ino    # Respaldo
```

## 🚀 API REST - Endpoint Unificado

### `/activate` - Control Unificado (POST)

Controla relés tanto del master (1-16) como del esclavo (17-32) con soporte para activación temporal.

#### Parámetros JSON:
```json
{
  "relay": 1-32,      // Número de relé (1-16: master, 17-32: esclavo)
  "state": 0-1,       // Estado (0: OFF, 1: ON)
  "delay": 0-∞        // Delay en microsegundos (0: permanente)
}
```

#### Mapeo de Relés:
- **Relés 1-16**: Control local (master) vía I2C
- **Relés 17-32**: Control remoto (esclavo) vía RS485

#### Ejemplos de Uso:

**✅ Activar relé 8 del master por 8 segundos:**
```bash
curl -X POST http://192.168.2.100/activate \
  -H "Content-Type: application/json" \
  -d '{"relay":8,"state":1,"delay":8000000}'
```

**✅ Activar relé 20 del esclavo permanentemente:**
```bash
curl -X POST http://192.168.2.100/activate \
  -H "Content-Type: application/json" \
  -d '{"relay":20,"state":1,"delay":0}'
```

**✅ Apagar relé 25 del esclavo:**
```bash
curl -X POST http://192.168.2.100/activate \
  -H "Content-Type: application/json" \
  -d '{"relay":25,"state":0,"delay":0}'
```

### `/status` - Estado del Sistema (GET)

Consulta el estado actual de todos los relés del master.

```bash
curl http://192.168.2.100/status
```

**Respuesta:**
```json
{
  "relays": [0,1,0,0,1,0,0,0,1,0,0,0,0,0,0,0],
  "message": "Estado actual de 16 relés"
}
```

## 🧪 Colección Postman

### Configuración Base:
- **Base URL**: `http://192.168.2.100`
- **Content-Type**: `application/json`

### Ejemplos de Requests:

#### 1️⃣ Activar Relé Master (1-16)
```
POST http://192.168.2.100/activate
Headers:
  Content-Type: application/json
Body (raw):
  {"relay":8,"state":1,"delay":8000000}
Descripción: Encender relé 8 master por 8 segundos
```

#### 2️⃣ Activar Relé Esclavo (17-32)
```
POST http://192.168.2.100/activate
Headers:
  Content-Type: application/json
Body (raw):
  {"relay":20,"state":1,"delay":0}
Descripción: Encender relé 4 esclavo permanente
```

#### 3️⃣ Activar Último Relé Esclavo
```
POST http://192.168.2.100/activate
Headers:
  Content-Type: application/json
Body (raw):
  {"relay":32,"state":1,"delay":5000000}
Descripción: Encender relé 16 esclavo por 5 segundos
```

#### 4️⃣ Desactivar Cualquier Relé
```
POST http://192.168.2.100/activate
Headers:
  Content-Type: application/json
Body (raw):
  {"relay":25,"state":0,"delay":0}
Descripción: Apagar relé 9 esclavo
```

#### 5️⃣ Estado del Sistema
```
GET http://192.168.2.100/status
Descripción: Consultar estado de todos los relés
```

## 🔧 Parámetros de Delay

### Conversión Automática:
- **Input**: Microsegundos (μs)
- **Procesamiento**: Conversión automática a milisegundos (ms)
- **Delay = 0**: Control permanente (sin apagado automático)
- **Delay > 0**: Activación temporal con apagado automático

### Ejemplos de Conversión:
```
1,000,000 μs = 1,000 ms = 1 segundo
5,000,000 μs = 5,000 ms = 5 segundos
8,000,000 μs = 8,000 ms = 8 segundos
```

## 🛠️ Instalación y Configuración

### 1. Preparación del Hardware:
- Conectar Ethernet al KC868-A16 master
- Conectar RS485 entre master y esclavo (A-A, B-B)
- Verificar alimentación de ambos módulos

### 2. Programación:
- Cargar `KC868_A16_Ethernet_Fixed.ino` en el master
- Cargar `KC868_A16_RS485_Slave.ino` en el esclavo
- Configurar Arduino IDE con librerías ESP32

### 3. Librerías Requeridas:
```cpp
#include <Wire.h>          // I2C communication
#include <ETH.h>           // Ethernet support
#include <WebServer.h>     // HTTP server
#include <ArduinoJson.h>   // JSON parsing
#include <HardwareSerial.h> // RS485 communication
```

### 4. Verificación:
- Comprobar conectividad Ethernet: `ping 192.168.2.100`
- Probar endpoint status: `curl http://192.168.2.100/status`
- Verificar comunicación RS485 con relés 17-32

## 📊 Protocolo RS485

### Comunicación Master → Esclavo:
```
Formato: RELAY:XX:Y
- XX: Número de relé esclavo (01-16)
- Y: Estado (0=OFF, 1=ON)

Ejemplo: "RELAY:03:1" → Activar relé 3 del esclavo
```

### Respuesta Esclavo → Master:
```
Formato: ACK:XX:Y o ERR:XX
- ACK: Comando ejecutado correctamente
- ERR: Error en la ejecución

Ejemplo: "ACK:03:1" → Relé 3 activado correctamente
```

## 🔍 Troubleshooting

### Problema: Ethernet no funciona
**Solución**: Verificar que no hay conflictos de GPIO, especialmente GPIO17

### Problema: RS485 no comunica
**Solución**: 
- Verificar conexiones A-A, B-B
- Confirmar mismo baudrate (9600) en master y esclavo
- Revisar pinout oficial KC868-A16

### Problema: Relés no responden
**Solución**:
- Verificar alimentación I2C
- Confirmar direcciones 0x24 y 0x25
- Revisar cableado SDA/SCL

### Problema: API retorna error 400
**Solución**:
- Verificar formato JSON correcto
- Confirmar Content-Type: application/json
- Validar rango de relés (1-32)

## 📝 Logs y Debug

### Activar Debug Serial:
```cpp
Serial.begin(115200);  // Ya incluido en el código
```

### Monitoreo en tiempo real:
- Abrir Serial Monitor en Arduino IDE
- Baudrate: 115200
- Observar logs de conexión Ethernet, RS485 y activación de relés

## 🔄 Compatibilidad

### Endpoints Legacy:
- `/relay`: Mantiene compatibilidad con versiones anteriores
- Solo controla relés master (1-16)
- Sin soporte para delay automático

### Migración a `/activate`:
- Reemplazar todas las llamadas `/relay` por `/activate`
- Agregar parámetro `delay` según necesidades
- Actualizar numeración si se usan relés esclavo

## 🎯 Casos de Uso

### 1. Control Industrial:
- Activación secuencial de máquinas
- Control de iluminación con temporizadores
- Automatización de procesos

### 2. Domótica:
- Control de electrodomésticos
- Sistemas de riego automático
- Gestión de climatización

### 3. Seguridad:
- Control de accesos
- Activación de alarmas
- Sistemas de emergencia

---

## 📞 Soporte

Para reportar issues o solicitar features:
1. Revisar logs del Serial Monitor
2. Verificar configuración de red
3. Confirmar pinout de hardware
4. Documentar el comportamiento observado

**¡Sistema listo para producción!** 🚀