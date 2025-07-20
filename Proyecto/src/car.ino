/*
 * ========================================================================================================
 * Proyecto: Carro IoT controlado por app (Blynk) con ESP32-S3
 * Curso: IE-0624 Laboratorio de Microcontroladores
 * 
 * Estudiantes:
 *      - Fabian Sander Hangen, C07336
 *      - Kristhel Quesada López, C06153
 * 
 * Microcontrolador:
 *      - Adafruit ESP32-S3 Feather
 * 
 * Descripción del Código:
 *      - Este código implementa el control remoto de un carro robótico mediante la app Blynk.
 *      - El ESP32-S3 se conecta a una red WiFi de 2.4 GHz y recibe comandos desde la app móvil.
 *      - El carro se controla a través de un módulo L298N que maneja 4 pines de control (IN1–IN4).
 *      - Se utilizan pines virtuales en Blynk (V1–V4) para avanzar, retroceder y girar.
 *      - Se incluyen fragmentos de código comentados para lectura de sensor ultrasónico y voltaje,
 *        pero no están activos en esta versión, ya que esas funciones las ejecuta otro microcontrolador
 *        (Arduino Uno).
 * 
 * Estado actual:
 *      - Control funcional de los motores mediante app Blynk.
 *      - Sensor ultrasónico y lectura de batería no implementados en esta versión.
 * 
 * Requisitos:
 *      - Red WiFi de 2.4 GHz
 *      - App Blynk instalada y vinculada al template correspondiente
 * 
 * Fecha: Julio 2025
 * ========================================================================================================
 */


// Identificadores del proyecto en Blynk
#define BLYNK_TEMPLATE_ID "TMPL2BFLMBXUy"
#define BLYNK_TEMPLATE_NAME "Esp32 IoT car"
#define BLYNK_AUTH_TOKEN "set_token_generated_by_blynk"

#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

// Credenciales WiFi
char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "set_your_wifi_network_must_be_2.4GHz";
char pass[] = "set_your_password_here";

// Pines de control de motores (conectados al L298N)
int IN1 = 5;
int IN2 = 6;
int IN3 = 9;
int IN4 = 13;

// Pines del sensor ultrasónico (NO USADOS EN ESTA VERSIÓN)
// int trig = 10;
// int echo = 11;

// Pin de lectura de batería (NO USADO)
// #define BATTERY_PIN 12

void setup() {
  Serial.begin(115200);

  // Configurar pines de motores como salida
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // ----- Pines no utilizados -----
  // pinMode(trig, OUTPUT);
  // pinMode(echo, INPUT);
  // -------------------------------

  // Configuración de red WiFi y conexión con Blynk
  Blynk.begin(auth, ssid, pass);
}

// Controles de movimiento desde la app Blynk
BLYNK_WRITE(V1) { digitalWrite(IN2, param.asInt()); digitalWrite(IN4, param.asInt()); }  // Reversa
BLYNK_WRITE(V2) { digitalWrite(IN1, param.asInt()); digitalWrite(IN3, param.asInt()); }  // Drive
BLYNK_WRITE(V3) { digitalWrite(IN2, param.asInt()); digitalWrite(IN3, param.asInt()); }  // Turn left
BLYNK_WRITE(V4) { digitalWrite(IN1, param.asInt()); digitalWrite(IN4, param.asInt()); }  // Turn right

// Función que se ejecuta siempre de manera indefinida
void loop() {
  // Ejecutar las tareas de Blynk
  Blynk.run();

  // -----------------------------------------------------------
  // Funcionalidad No Implementada
  // -----------------------------------------------------------
  //if (millis() - lastSensorRead > 500) {
  //  lastSensorRead = millis();
  //
  //  // --- Lectura de distancia para V5 ---
  //  digitalWrite(trig, LOW);
  //  delayMicroseconds(2);
  //  digitalWrite(trig, HIGH);
  //  delayMicroseconds(10);
  //  digitalWrite(trig, LOW);
  //
  //  long duracion = pulseIn(echo, HIGH, 30000);
  //  int distancia = (duracion == 0) ? 0 : duracion * 0.0343 / 2;
  //
  //  Serial.print("Distancia: ");
  //  Serial.print(distancia);
  //  Serial.println(" cm");
  //
  //  if (distancia > 0 && distancia < 10) {
  //    Blynk.virtualWrite(V5, 255);  // LED encendido
  //  } else {
  //    Blynk.virtualWrite(V5, 0);    // LED apagado
  //  }
  //
  //  // --- Lectura del voltaje de batería para A1 ---
  //  int raw = analogRead(BATTERY_PIN);
  //  float voltaje = raw * (3.3 / 4095.0);
  //
  //  Serial.print("Voltaje batería: ");
  //  Serial.print(voltaje);
  //  Serial.println(" V");
  //
  //  Blynk.virtualWrite(A1, voltaje);
  //}
}
