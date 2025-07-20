/*
 * ==============================================================================
 * Proyecto: Medición de distancia con sensor ultrasónico y activación de alarma
 * Curso: IE-0624 Laboratorio de Microcontroladores
 * 
 * Estudiantes:
 *   - Fabian Sander Hangen, C07336
 *   - Kristhel Quesada López, C06153
 *
 * Microcontrolador:
 *   - Arduino Uno
 * 
 * Descripción del Código:
 *   - El sistema mide la distancia utilizando un sensor ultrasónico HC-SR04.
 *   - Si un objeto se detecta a menos de 10 cm, se activa una alarma conectada
 *     al pin especificado.
 * 
 * Componentes:
 *   - Sensor ultrasónico HY-SRF05
 *   - Alarma (zumbador, LED u otro actuador) conectada a un pin digital
 * 
 * Fecha: Julio 2025
 * ==============================================================================
 */

// Declaracion de los pines
int trig = 7;                 // Pin de disparo del sensor ultrasónico
int echo = 6;                 // Pin de eco del sensor ultrasónico
int alarma = 8;               // Pin de salida para la alarma (por ejemplo, un zumbador)


// Declaracion de variables
long tiempoMicro;             // Tiempo de ida y vuelta de la señal en microsegundos
float distanciaCm;            // Distancia calculada en centímetros


// Configuracion inicial
void setup() {
  Serial.begin(9600);         // Inicia la comunicación serial
  pinMode(trig, OUTPUT);      // Configura el pin de disparo como salida
  pinMode(echo, INPUT);       // Configura el pin de eco como entrada
  pinMode(alarma, OUTPUT);    // Configura el pin de la alarma como salida
}

// Función que se ejecuta siempre de manera indefinida
void loop() {
  // Enviar un pulso al sensor ultrasónico
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  // Medir duración del pulso de eco
  tiempoMicro = pulseIn(echo, HIGH);

  // Calcular distancia en centímetros
  distanciaCm = (tiempoMicro * 0.0343) / 2;

  // Mostrar distancia por el monitor serial
  Serial.print("Distancia: ");
  Serial.print(distanciaCm);
  Serial.println(" cm");

  // Activar o desactivar la alarma según la distancia
  if (distanciaCm > 0 && distanciaCm < 10) {
    digitalWrite(alarma, HIGH);  // Enciende alarma si el objeto está muy cerca
  } else {
    digitalWrite(alarma, LOW);   // Apaga alarma
  }

  delay(100);  // Esperar 100 ms antes de repetir
}
