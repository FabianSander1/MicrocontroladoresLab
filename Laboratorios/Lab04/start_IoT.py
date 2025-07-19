#!/usr/bin/env python3
# ==============================================================================
# Laboratorio 04: STM32 con uso de GPIO, ADC, comunicaciones y Iot
# IE-0624 Laboratorio de Microcontroladores
# 
# Estudiantes:
#      - Fabian Sander Hangen, C07336
#      - Kristhel Quesada Lopez, C06153
# 
# Microcontrolador:
#      - STM32F429 Discovery Kit
# 
# Descripcion del Codigo:
#      - Sismografo que mida el nivel de una batería y los valores del giros-
#        copio, y despliegue los resultados en la pantalla LCD del MCU. A su
#        vez, debe enviar los resultados por USART para que sean procesados
#        por un archivo .py y enviados a la plataforma de Thingsboard.
# ==============================================================================

import serial                    # Librería para comunicación por puerto serial
import json                      # Para trabajar con datos en formato JSON
import paho.mqtt.client as mqtt  # Cliente MQTT para comunicación con el broker



# -------------------- Configuración --------------------

SERIAL_PORT = "/dev/ttyACM0"        # Puerto serial
BAUDRATE = 115200                   # Velocidad en la ransmisión de datos

# Configuración del broker MQTT
BROKER = "iot.eie.ucr.ac.cr"        # Dirección del broker
PORT = 1883                         # Puerto del broker
TOKEN = "fjw35yyuzs2qcs2gb8in"      # Token de autenticación
TOPIC = "v1/devices/me/telemetry"   # Tema donde se publicarán los datos




# -------------------- Inicialización del puerto serial --------------------
try:
    # Abre el puerto serial con la configuración dada
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
    print("Puerto serial abierto correctamente:", SERIAL_PORT)
except serial.SerialException as e:
    # Error al abrir el puerto serial
    print("Error abriendo el puerto serial:", e)
    exit()



# -------------------- Inicialización del cliente MQTT --------------------

# Crea una instancia del cliente MQTT
client = mqtt.Client(protocol=mqtt.MQTTv311)
# Establece el token como nombre de usuario (no requiere contraseña)
client.username_pw_set(TOKEN)

try:
    # Intenta conectar con el broker MQTT
    client.connect(BROKER, PORT)
    print("Conectado al broker MQTT")
except Exception as e:
    # Error al conectar con el broker
    print("Error conectando con el broker:", e)
    exit()

# Inicia el loop del cliente MQTT en segundo plano para mantener la conexión activa
client.loop_start()



# -------------------- Bucle principal --------------------
while True:
    try:
        # Lee una línea de datos del puerto serial
        raw_line = ser.readline().decode('utf-8', errors='ignore').strip()
        
        # Si la línea está vacía, continúa con la siguiente iteración
        if not raw_line:
            continue

        print("\nLeído:", raw_line)

        # Se espera que los datos estén separados por tabulaciones: x	y	z	batería
        parts = raw_line.split('\t')
        
        # Verifica que la línea tenga exactamente 4 partes
        if len(parts) != 4:
            print("[ERROR] Formato inesperado:", parts)
            continue

        # Convierte los tres primeros valores a enteros (x, y, z) y el último a flotante (batería)
        x, y, z = map(int, parts[:3])
        battery = float(parts[3])

        # Crea un diccionario con los datos
        payload = {
            "x": x,
            "y": y,
            "z": z,
            "battery": battery
        }

        print("Enviando:", payload)
        
        # Publica los datos en formato JSON al tópico especificado
        client.publish(TOPIC, json.dumps(payload))

    except Exception as e:
        # Captura cualquier error ocurrido durante el procesamiento de la línea
        print("Error procesando línea:", e)
