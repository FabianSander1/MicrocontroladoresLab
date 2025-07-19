#include <pic14/pic12f683.h>
#include <stdint.h>

// Configuración del microcontrolador
typedef unsigned int word;
word __at 0x2007 __CONFIG = (_WDTE_OFF);

// Variable global para almacenar el número actual
volatile uint8_t numeroActual = 1; 

uint8_t rand_con_timer(void) {
    static uint8_t semilla = 0x5A; // Semilla inicial arbitraria
    semilla ^= TMR0;               // Mezcla con el valor actual del temporizador
    semilla = (semilla << 1) | (semilla >> 7); // Rotación de bits
    uint8_t r = (semilla % 6) + 1; // Limita el rango entre 1 y 6
    return r;
}


// Función para generar una pausa en la ejecución
void delay(unsigned int intervaloTiempo) {
    unsigned int contadorExterno, contadorInterno;
    for (contadorExterno = 0; contadorExterno < intervaloTiempo; contadorExterno++)
        for (contadorInterno = 0; contadorInterno < 1275; contadorInterno++);
}

// Función principal del programa
void main(void) {
    TRISIO = 0b00100000; // Configura los pines, GP5 como entrada y el resto como salida
    GPIO = 0x00; // Inicializa todos los pines en bajo
    
    OPTION_REG = 0b00000111;
    
    unsigned int tiempoEspera = 200;
    unsigned int numeroAleatorio = 0;

    while (1) {
        if (GP5) { // Si el botón en GP5 está presionado
            numeroAleatorio = rand_con_timer(); // Genera un número aleatorio entre 1 y 6

            switch (numeroAleatorio) {
                case 1:
                    GP1 = 1; // Enciende el LED en GP1
                    delay(tiempoEspera);
                    GP1 = 0; // Apaga el LED
                    break;
                
                case 2:
                    GP0 = 1; // Enciende el LED en GP0
                    delay(tiempoEspera);
                    GP0 = 0;
                    break;
                
                case 3:
                    GPIO = 0b00000011; // Enciende dos LEDs
                    delay(tiempoEspera);
                    GPIO = 0b00000000;
                    break;
                
                case 4:
                    GPIO = 0b00000101; // Enciende tres LEDs
                    delay(tiempoEspera);
                    GPIO = 0b00000000;
                    break;
                
                case 5:
                    GPIO = 0b00000111; // Enciende cuatro LEDs
                    delay(tiempoEspera);
                    GPIO = 0b00000000;
                    break;
                
                case 6:
                    GPIO = 0b00010101; // Enciende cinco LEDs
                    delay(tiempoEspera);
                    GPIO = 0b00000000;
                    break;
            }
        } else {
            GPIO = 0x00; // Si no se presiona el botón, apaga todos los LEDs
        }
    }
}
