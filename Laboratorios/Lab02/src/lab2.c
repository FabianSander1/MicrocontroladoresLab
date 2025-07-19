/*
 * ==============================================================================
 * Laboratorio 02: GPIOs, Timers y FSM
 * IE-0624 Laboratorio de Microcontroladores
 * 
 * Estudiantes:
 *      - Fabian Sander Hangen,
 *      - Kristhel Quesada Lopez, C06153
 * 
 * Microcontrolador:
 *      - ATtiny4313
 * 
 * Descripcion del Codigo:
 *      - Simulacion del comportamiento de un cruce de semaforo con señalizacion
 *        vehicular y peatonal interactiva.
 * ==============================================================================
*/


/* 
 * ------------------------------------------
 * Archivos de encabezado del AVR MCU
 * ------------------------------------------
*/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>



/* 
 * ------------------------------------------
 * Configuracion de los macros
 * ------------------------------------------
*/
#define F_CPU 8000000UL     // Frecuencia del CLK (8 MHz)

#define LVV PB0             // Salida LED verde vehiculo
#define LRV PB1             // Salida LED rojo vehiculo
#define LRP PB3             // Salida LED rojo peaton
#define LVP PB2             // Salida LED verde peaton
#define BTN PD2             // ENtrada boton peatonal



/* 
 * ------------------------------------------
 * Declaracion de estados para la FSM
 * ------------------------------------------
*/
typedef enum {
    VEH_PASS,    // Paso vehículos (verde para vehículos, rojo para peatones)
    VEH_BLINK,   // Parpadeo de verde vehículo
    RED_BUFFER1, // Rojo para vehículos y peatones (transición)
    PED_PASS,    // Paso peatones (rojo vehículos, verde peatón)
    PED_BLINK,   // Parpadeo verde peatón
    RED_BUFFER2  // Rojo para vehículos y peatones (transición)
} State;



/* 
 * ------------------------------------------
 * Variables globales
 * ------------------------------------------
*/
uint8_t state_timer;             // Contador de tiempo para los estados
volatile State state;            // Estado actual
volatile bool second_tick;       // Indicador de que 1 segundo ha transcurrido
volatile bool button_pressed;    // Indicador de que el boton fue presionaldo
volatile bool people_waiting;    // Indicador de que hay personas esperando (<10s)
uint8_t people_timer;            // Timer de cuanto tiempo llevan las personas esperando



/* 
 * ------------------------------------------
 * Inicializacion de interrupciones
 *  - Basado en los macro de interrupt.h
 *  - Se utilizan los vectores de iotn4313.h
 * ------------------------------------------
*/
// Interrupcion de flanco o nivel
ISR(INT0_vect) {
    button_pressed = true;
}

// Interrupción de temporizador (cada 1s aprox)
ISR(TIMER1_COMPA_vect) {
    second_tick = true;
    state_timer++;

    if (people_waiting) {
        people_timer++;
    }
}



/* 
 * ------------------------------------------
 * Funcion de setup
 * ------------------------------------------
*/
void setup() {
    // Inicializacion de variables globales
    state_timer = 0;               // Timer inicia en cero
    state = VEH_PASS;              // Esado inicial en paso vehicular
    second_tick = false;           // Flag inicia desactivada
    button_pressed = false;        // Flag inicia desactivada
    people_waiting = false;        // Flag inicia desactivada
    people_timer = 0;              // Timer inicia en cero

    // Configuración de los LEDs (pines) como salidas
    // si es 1 es salida
    // si es 0 es entrada
    DDRB |= (1 << LVV) | (1 << LRV) | (1 << LVP) | (1 << LRP);

    // Configuración del botón como entrada con pull-up
    DDRD &= ~(1 << BTN);            
    PORTD |= (1 << BTN);                  // para la configuracion de resistencia pull up

    // Configuración de la interrupción por flanco de bajada en el botón
    MCUCR |= (1 << ISC01);                // se configura ISC01 para el modo falling edge en el registro MCUCR
    GIMSK |= (1 << INT0);                 // habilita la interrupción externa de INT0 correspondiente al pin PD2

    // Configuración del temporizador 1 para interrupciones cada 1 segundo
    TCCR1A = 0;                    // Reafirmamos condicion inicial
    TCCR1B |= (1 << WGM12);        //Modo CTC, relacionado con el TCCR1A de arriba, WGM12 corresponde al bit 3 de TCCR1B
    TCCR1B |= (1 << CS12);         // Prescaler 256, reduce la frecuencia del clk a una razon de 256
    OCR1A = 31249;                 // Valor de comparación que reinicia el counter cuando se matchea este valor (pag. 98)
    TIMSK |= (1 << OCIE1A);        // Habilitar interrupción por comparación A (registro previamente definido)

    sei();                         // Habilitar interrupciones globales, proviene del archivo interrupt.h
}



/* 
 * ------------------------------------------
 * Funcion Principal
 * ------------------------------------------
*/
int main()
{
    setup();

    while (1) {
        // Verificar si ha transcurrido un segundo
        if (second_tick) {
            second_tick = false;  // Restablecer el indicador para que inicie la cuenta nuevamente

            // Comportamiento dependiendo del estado actual
            switch (state) {

                case VEH_PASS:
                    PORTB |= (1 << LVV);                  // Verde para vehículos
                    PORTB |= (1 << LRP);                  // Rojo para peatones
                    PORTB &= ~((1 << LRV) | (1 << LVP));  // Apagar rojo vehículo y verde peatón

                    // Si se presiona el botón, no habian personas esperando y han pasado menos de 10s
                    if (button_pressed && !people_waiting && state_timer <= 10) {
                        button_pressed = false;           // Apagamos la bandera de interrupcion
                        people_waiting = true;            // Activamos bandera de personas esperando
                        people_timer = 0;                 // Contador de espera
                    }
                
                    // Si ya se está esperando desde que se presionó el botón
                    if ((people_waiting && people_timer > 10) || (button_pressed && state_timer > 10)) {
                        button_pressed = false;           // Apagamos la bandera de interrupcion
                        people_waiting = false;           // Apagamos la bandera de personas esperando
                        people_timer = 0;                 // Reseteamos contador de espera
                        state_timer = 0;                  // Reseteamos contador de estado
                        state = VEH_BLINK;
                    }
                
                    break;

                case VEH_BLINK:
                    // Siempre dentro de este estado, toggle el valor de luz verde vehicular
                    PORTB ^= (1 << LVV);

                    // Transición a RED_BUFFER1 después de 3 blinks
                    if (state_timer > 6) {
                        state_timer = 0;
                        state = RED_BUFFER1;
                    }
                    break;


                // Delay simula "luz amarilla" para no generar transiciones abruptas
                case RED_BUFFER1:
                    PORTB |= (1 << LRV);                 // Rojo para vehículos
                    PORTB |= (1 << LRP);                 // Rojo para peatones
                    PORTB &= ~((1 << LVV) | (1 << LVP)); // Apagar verde peatón y vehicular

                    if (state_timer > 1) { 
                        state_timer = 0;
                        state = PED_PASS;
                    }
                    break;

                case PED_PASS:
                    PORTB |= (1 << LRV);                 // Rojo para vehículos
                    PORTB |= (1 << LVP);                 // Verde para peatones
                    PORTB &= ~((1 << LVV) | (1 << LRP)); // Apagar rojo peatón y verde vehículos

                    if (state_timer > 10) { 
                        state_timer = 0;
                        state = PED_BLINK;
                    }
                    break;

                case PED_BLINK:
                    // Siempre dentro de este estado, toggle el valor de luz verde peatonal
                    PORTB ^= (1 << LVP); 

                    if (state_timer > 6) {
                        state_timer = 0;
                        state = RED_BUFFER2;
                    }
                    break;

                case RED_BUFFER2:
                    PORTB |= (1 << LRV);                 // Rojo para vehículos
                    PORTB |= (1 << LRP);                 // Rojo para peatones
                    PORTB &= ~((1 << LVV) | (1 << LVP)); // Apagar verde peatón y vehicular

                    if (state_timer > 1) { 
                        state_timer = 0;
                        state = VEH_PASS;
                    }
                    break;
            }
        }

    }
}
