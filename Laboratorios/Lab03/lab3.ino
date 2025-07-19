/*
 * ==============================================================================
 * Laboratorio 02: GPIO, ADC y protocolos de comunicación en Arduino
 * IE-0624 Laboratorio de Microcontroladores
 * 
 * Estudiantes:
 *      - Fabian Sander Hangen, C07336
 *      - Kristhel Quesada Lopez, C06153
 * 
 * Microcontrolador:
 *      - Arduino UNO con ATMega16
 * 
 * Descripcion del Codigo:
 *      - Voltimetro que permite la lectura de cuatro canales para valores de
 *        voltaje analogicos en el rango de [-24, 24]V con warnings visuales a
 *        partir de los [-20, 20]V. Modos de lectura DC y AC (rms).
 * ==============================================================================
*/


/* 
 * ----------------------------------------------------------------
 * Archivos de encabezado
 * ----------------------------------------------------------------
*/
#include <SoftwareSerial.h>
#include <PCD8544.h>
#include <math.h>



/* 
 * ----------------------------------------------------------------
 * Configuracion de los macros
 * ----------------------------------------------------------------
*/
// Pines analógicos de entrada
const byte CANALES_ADC[] = {A0, A1, A2, A3};

// Pines de digitales
const byte PIN_LED = 8;             // Controla el warning LED
const byte PIN_SERIAL_EN = 9;       // Lee habilitación de com. serial
const byte PIN_MEASURE_MODE = A4;   // Lee el modo de medida (1=Ac, 0=DC)

// Offset aplicadp
const float gain = 1/0.104;         // 9.6154
const float offset = 2.5;

// Inicialización de un objeto PCD8544 (LCD display)
PCD8544 display;



/* 
 * ----------------------------------------------------------------
 * Funcion Setup
 *  - Configura el estado inicial del sistema
 *  - Inicia el objeto Display para su preparacion y uso
 *  - Inicia la comunicación serial
 *  - Configura los pines I/O
 * ----------------------------------------------------------------
*/
void setup() {
    display.begin();                // Inicializa el uso del display LCD
    Serial.begin(9600);             // Inicializa com. serial a 9600 baudios    
    pinMode(PIN_LED, OUTPUT);       // Configura salida digital
    pinMode(PIN_SERIAL_EN, INPUT);  // COnfigura entrada digital
}



/* 
 * ----------------------------------------------------------------
 * Funcion Loop
 *  - Ejecucion ciclica
 *  - Procesa la información segun el modo de operación (AC o DC)
 * ----------------------------------------------------------------
*/
void loop() {
    int lecturaModo = analogRead(PIN_MEASURE_MODE); // Lee valor de pin A4
    bool modo = lecturaModo > 512;                  // 1 si AC, 0 de lo contrario
    procesarCanales(modo);                          // Procesa los canales segun el modo
    delay(1000);                                    // Espera, para volver a leer
}



/* 
 * ----------------------------------------------------------------
 * Funcion procesarCanales
 *  - Recibe la señal de tensión de 0-5V de los canales
 *  - Imprime el tipo de modo en el display
 *  - Calcula el voltaje segun el modo
 *  - Muestra el resultado del voltaje para cada canal
 *  - Checkea si la comunicación serial está habilitada para enviar
 *    los datos
 * ----------------------------------------------------------------
*/
void procesarCanales(bool esAC) {
    // Logica por modo de operacion (AC o DC)
    const char* label = esAC ? "Modo AC" : "Modo DC";          // Puntero que guarda una etiqueta del modo
    float (*getVoltage)(byte) = esAC ? volts2rms : volts2dc;   // Puntero a una funcion

    // Variables/flags para logica del warning LED
    bool outRange = 0;
    float umbral = esAC ? 14.14 : 20.0;

    // Configuracion inicial del display
    display.clear();                                           // Limpia la info de la pantalla
    display.setCursor(0, 0);                                   // Inicia punto de nueva escritura
    display.print(label);                                      // Escribe el modo de medida

    if (digitalRead(PIN_SERIAL_EN) == HIGH) {
        Serial.println(label);
    }

    // Imprimir el voltaje por canal
    for (byte i = 0; i < 4; i++) {
        float volts = getVoltage(i);                           // Obtener voltaje del canal

        outRange = (fabs(volts) > umbral) ? 1 : outRange;      // Bandera de warning de voltaje
        mostrarLCD(i, volts);                                  // Imprime canal y voltaje en el display

        // Si el serial enable está activo, imprima por serial
        if (digitalRead(PIN_SERIAL_EN) == HIGH) {
            enviarSerial(i, volts);
        }
    }
    
    // Enciede warning LED si es necesario
    controlarLED(outRange);
}



/* 
 * ----------------------------------------------------------------
 * Funcion volts2dc
 *  - Lee voltaje ADC y convierte a voltaje real escalado DC
 * ----------------------------------------------------------------
*/
float volts2dc(byte canal) {
    float adc = analogRead(CANALES_ADC[canal]);   // Vanalogico 0-1023
    float rawVolts = (adc / 1023.0) * 5.0;        // [0, 5] V
    float centered = rawVolts - offset;           // [-5/2, 5/2] V
    float realValue = centered * gain;            // [-24, 24] V
    return realValue;
}



/* 
 * ----------------------------------------------------------------
 * Funcion volts2rms
 *  - Lee el voltaje de canal y retorna valor en RMS
 * ----------------------------------------------------------------
*/
float volts2rms(byte canal) {
    const int numSamples = 300;                     // Cantidad de samples 
    float samples[numSamples];                      // Array para guardar los samples

    for (int i = 0; i < numSamples; i++) {
        float adc = analogRead(CANALES_ADC[canal]); // 0-1023
        float rawVolts = (adc / 1023.0) * 5.0;      // [0, 5]V
        float centered = rawVolts - offset;         // [-5/2, 5/2]
        float realVolts = centered * gain;          // [-24, 24]

        samples[i] = realVolts;                     // Guardar sample en array
        delayMicroseconds(56);                      // Para 300 muestras a 60Hz
    }

    // Obtener mínimo y máximo
    float vmin = samples[0];
    float vmax = samples[0];
    for (int i = 1; i < numSamples; i++) {
        if (samples[i] < vmin) vmin = samples[i];
        if (samples[i] > vmax) vmax = samples[i];
    }

    // Calculo RMS
    float vrms;
    if (vmax > 0) vrms = vmax / sqrt(2.0);          // Señal AC positiva, entonces...
    else if (vmax < 0) vrms = vmin / sqrt(2.0);     // Señal AC negativa, entonces...
    else vrms = 0;                                  // Es cero

    // DEBUG prints, descomente para debuggear
    //Serial.print("Canal "); Serial.print(canal);
    //Serial.print(" - Vmin: "); Serial.print(vmin);
    //Serial.print(" Vmax: "); Serial.print(vmax);
    //Serial.print(" RMS: "); Serial.println(vrms);

    return vrms;
}



/* 
 * ----------------------------------------------------------------
 * Funcion mostrarLCD
 *  - Configura el display LCD, escribiendo los valores que se
 *    pasan por parametro con un formato especifico, para mostrar
 *    los valores en pantalla.
 * ----------------------------------------------------------------
*/
void mostrarLCD(byte canal, float valor) {
    display.setCursor(0, canal + 1);
    display.print("C");
    display.print(canal + 1);
    display.print(": ");
    display.print(valor);
    display.print("V");
}



/* 
 * ----------------------------------------------------------------
 * Funcion controlarLED
 *  - Enciende el LED si algun voltaje excede los parametros de
 *    seguridad
 * ----------------------------------------------------------------
*/
void controlarLED(bool turnOn) {
    digitalWrite(PIN_LED, turnOn ? HIGH : LOW);
}



/* 
 * ----------------------------------------------------------------
 * Funcion enviarSerial
 *  - Controla el envío de datos por serial.
 * ----------------------------------------------------------------
*/
void enviarSerial(byte canal, float volts) {
        Serial.print("Canal ");
        Serial.print(canal + 1);
        Serial.print(": ");
        Serial.print(volts);
        Serial.println(" V");
}
