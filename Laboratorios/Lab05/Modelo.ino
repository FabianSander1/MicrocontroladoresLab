/*
 * ==============================================================================
 * Laboratorio 05: Reconocimiento de Actividad Humana con Edge Impulse
 * IE-0624 Laboratorio de Microcontroladores
 * 
 * Estudiantes:
 *      - Fabian Sander Hangen, C07336
 *      - [Nombre del segundo estudiante], [Carné]
 * 
 * Microcontrolador:
 *      - Arduino Nano 33 BLE Sense
 * 
 * Descripción del Código:
 *      - Captura datos del acelerómetro interno, realiza inferencia utilizando
 *        un modelo entrenado con Edge Impulse y muestra las predicciones por 
 *        puerto serial. El sistema clasifica el tipo de movimiento humano en 
 *        tiempo real y actualiza los resultados suavizados.
 * ==============================================================================
*/

/*
 * ----------------------------------------------------------------
 * Inclusión de bibliotecas
 * ----------------------------------------------------------------
*/
#include <fsanderh-project-1_inferencing.h>   // Modelo generado por Edge Impulse
#include <Arduino_LSM9DS1.h>                 // Librería para el sensor IMU integrado

/*
 * ----------------------------------------------------------------
 * Definición de macros y constantes
 * ----------------------------------------------------------------
*/
#define CONVERT_G_TO_MS2    9.80665f         // Conversión de G a m/s²
#define MAX_ACCEPTED_RANGE  2.0f             // Límite máximo de lectura del acelerómetro

/*
 * ----------------------------------------------------------------
 * Variables globales
 * ----------------------------------------------------------------
*/
static bool debug_nn = false;                                // Habilita impresión de características (debug)
static uint32_t run_inference_every_ms = 200;                // Tiempo entre inferencias (ms)
static rtos::Thread inference_thread(osPriorityLow);         // Hilo de ejecución para inferencia
static float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = {0};     // Buffer de lectura IMU
static float inference_buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE]; // Buffer para análisis

/*
 * ----------------------------------------------------------------
 * Prototipos de funciones
 * ----------------------------------------------------------------
*/
void run_inference_background();         // Hilo que ejecuta la inferencia
float ei_get_sign(float number);         // Devuelve el signo del número


/*
 * ----------------------------------------------------------------
 * Función setup
 *  - Inicializa el IMU y el hilo de inferencia
 * ----------------------------------------------------------------
*/
void setup()
{
    Serial.begin(115200);                        // Inicializa UART
    while (!Serial);                             // Espera a conexión serial
    Serial.println("Edge Impulse Inferencing Demo");

    if (!IMU.begin()) {
        ei_printf("Failed to initialize IMU!\r\n");
    } else {
        ei_printf("IMU initialized\r\n");
    }

    if (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != 3) {
        ei_printf("ERR: EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME should be equal to 3\n");
        return;
    }

    // Inicia el hilo en segundo plano para procesamiento continuo
    inference_thread.start(mbed::callback(&run_inference_background));
}


/*
 * ----------------------------------------------------------------
 * Función ei_get_sign
 *  - Devuelve el signo de un número como float
 * ----------------------------------------------------------------
*/
float ei_get_sign(float number) {
    return (number >= 0.0) ? 1.0 : -1.0;
}


/*
 * ----------------------------------------------------------------
 * Función run_inference_background
 *  - Ejecuta continuamente la inferencia, transforma la señal
 *    y muestra predicciones suavizadas.
 * ----------------------------------------------------------------
*/
void run_inference_background()
{
    delay((EI_CLASSIFIER_INTERVAL_MS * EI_CLASSIFIER_RAW_SAMPLE_COUNT) + 100);

    ei_classifier_smooth_t smooth;
    ei_classifier_smooth_init(&smooth, 10, 7, 0.8, 0.3);  // Parámetros de suavizado

    while (1) {
        memcpy(inference_buffer, buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE * sizeof(float));

        signal_t signal;
        int err = numpy::signal_from_buffer(inference_buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
        if (err != 0) {
            ei_printf("Failed to create signal from buffer (%d)\n", err);
            return;
        }

        ei_impulse_result_t result = {0};
        err = run_classifier(&signal, &result, debug_nn);
        if (err != EI_IMPULSE_OK) {
            ei_printf("ERR: Failed to run classifier (%d)\n", err);
            return;
        }

        ei_printf("Predictions ");
        ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
                  result.timing.dsp, result.timing.classification, result.timing.anomaly);
        ei_printf(": ");

        const char *prediction = ei_classifier_smooth_update(&smooth, &result);
        ei_printf("%s ", prediction);

        ei_printf(" [ ");
        for (size_t ix = 0; ix < smooth.count_size; ix++) {
            ei_printf("%u", smooth.count[ix]);
            if (ix != smooth.count_size + 1) {
                ei_printf(", ");
            } else {
                ei_printf(" ");
            }
        }
        ei_printf("]\n");

        delay(run_inference_every_ms);
    }

    ei_classifier_smooth_free(&smooth);
}


/*
 * ----------------------------------------------------------------
 * Función loop
 *  - Ejecuta continuamente la captura del acelerómetro, ajusta
 *    el buffer de entrada y lo convierte a unidades físicas
 * ----------------------------------------------------------------
*/
void loop()
{
    while (1) {
        uint64_t next_tick = micros() + (EI_CLASSIFIER_INTERVAL_MS * 1000);

        // Desplaza el buffer 3 posiciones para hacer espacio
        numpy::roll(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, -3);

        // Lee aceleración X, Y, Z y almacena al final del buffer
        IMU.readAcceleration(
            buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 3],
            buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 2],
            buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 1]
        );

        // Limita los valores a ±2G
        for (int i = 0; i < 3; i++) {
            if (fabs(buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 3 + i]) > MAX_ACCEPTED_RANGE) {
                buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 3 + i] = 
                    ei_get_sign(buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 3 + i]) * MAX_ACCEPTED_RANGE;
            }
        }

        // Convierte lecturas de G a m/s²
        buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 3] *= CONVERT_G_TO_MS2;
        buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 2] *= CONVERT_G_TO_MS2;
        buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 1] *= CONVERT_G_TO_MS2;

        // Espera hasta el siguiente instante de muestreo
        uint64_t time_to_wait = next_tick - micros();
        delay((int)floor((float)time_to_wait / 1000.0f));
        delayMicroseconds(time_to_wait % 1000);
    }
}


/*
 * ----------------------------------------------------------------
 * Validación de modelo
 *  - Verifica en tiempo de compilación que el modelo es para
 *    sensor de acelerómetro.
 * ----------------------------------------------------------------
*/
#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_ACCELEROMETER
#error "Invalid model for current sensor"
#endif
