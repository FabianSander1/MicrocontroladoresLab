/*
 * ==============================================================================
 * Laboratorio 04: STM32 con uso de GPIO, ADC, comunicaciones y Iot
 * IE-0624 Laboratorio de Microcontroladores
 * 
 * Estudiantes:
 *      - Fabian Sander Hangen, C07336
 *      - Kristhel Quesada Lopez, C06153
 * 
 * Microcontrolador:
 *      - STM32F429 Discovery Kit
 * 
 * Descripcion del Codigo:
 *      - Sismografo que mida el nivel de una batería y los valores del giros-
 *        copio, y despliegue los resultados en la pantalla LCD del MCU. A su
 *        vez, debe enviar los resultados por USART para que sean procesados
 *        por un archivo .py y enviados a la plataforma de Thingsboard.
 * ==============================================================================
*/

/* 
 * ----------------------------------------------------------------
 * Archivos de encabezado
 * ----------------------------------------------------------------
*/
// Librerías estándar de C
#include <stdio.h>      // [Incr1] Para sprintf
#include <stdint.h>     // Para tipos de datos fijos como uint8_t, int16_t

// Extraidos del directorio de ejemplos (lcd-serial)
#include "console.h"    // [Incr1] Inicializa la consola UART (console_setup)
#include "lcd-spi.h"    // [Incr1] Inicializa la pantalla LCD vía SPI (lcd_spi_init)
#include "clock.h"      // [Incr1] Configura el reloj del sistema (clock_setup)
#include "sdram.h"      // [Incr1] Inicializa la SDRAM externa (sdram_init)
#include "gfx.h"        // [Incr1] Funciones gráficas: texto, rectángulos, pantalla (gfx_puts, gfx_fillRect, etc.)
#include "adc.h"        // Lectura de voltaje desde el ADC (init_adc, read_voltage)
#include "usart.h"      // Configura USART para transmisión (init_uart, usart_send_blocking)

// General (libopencm3)
#include <libopencm3/stm32/gpio.h>  // GPIO: manejo de pines, entradas/salidas
#include <libopencm3/stm32/rcc.h>   // RCC: habilitación de relojes para periféricos
#include <libopencm3/stm32/spi.h>   // SPI: control de periféricos SPI a nivel de registro para el giroscopio



/* 
 * ----------------------------------------------------------------
 * Configuracion de los macros
 * ----------------------------------------------------------------
*/
// Macros para el giroscopio
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} GyroData;

// Macros para el SPI
#define CS_PORT GPIOC
#define CS_PIN  GPIO1

#define GYR_XL    0x28
#define GYR_XH    0x29
#define GYR_YL    0x2A
#define GYR_YH    0x2B
#define GYR_ZL    0x2C
#define GYR_ZH    0x2D
#define READ_FLAG 0x80
#define GYR_CTRL1 0x20
#define GYR_CTRL4 0x23

#define DPS       0b00      // 250dps
#define BW_SHIFT  4         // DR=75Hz y BW=12.5Hz
#define GYR_SENS  0.00875f  // Sensitivity for 200dps

// Macros para la bateria
#define BAT_PORT GPIOA
#define BAT_PIN  GPIO6
#define CHANNEL  6
#define MAX_VOLT 9.00f

// Macros para los LEDs (bateria baja y serial)
#define LEDS_PORT GPIOG
#define RLED_PIN  GPIO14
#define GLED_PIN  GPIO13

// Macros para el botón de habilitación USART
#define BTN_PORT GPIOA
#define BTN_PIN  GPIO0

// Macros para el uso de USART1 con Alternate Functions
#define USART1_PORT GPIOA
#define USART1_TX   GPIO9


/* 
 * ----------------------------------------------------------------
 * Prototipos de las Funciones
 * ----------------------------------------------------------------
*/
// Funciones de inicializacion
void init_lcd(void);
void init_spi(void);
void init_gpio(void);
void init_adc(void);
void init_usart(void);

// Funciones para el control del display en el LCD
void displayLCD(GyroData gyro, float voltage, int sending);

// Funciones para el control del giroscopio con SPI
uint8_t get_spi_read(uint8_t cmd);
int16_t read_gyro_axis(uint8_t lsb, uint8_t msb);
GyroData get_gyro_data(void);

// Funciones para el control de la lectura de la bateria
float read_voltage(uint8_t channel);

// Funciones para el control de los LEDs
void status_leds(int sending, float voltage);

// Funciones para el envio USART
void xmit_usart(GyroData g, float v);

// Funcion de setup
void setup(void);



/* 
 * ----------------------------------------------------------------
 * Funcion InitLCD
 *  - Inicializa las variables necesarias para habilitar el uso
 *    de la pantalla LCD
 * ----------------------------------------------------------------
*/
void init_lcd(void) {
    clock_setup();
    console_setup(115200);    				// Habilita consola por UART
    sdram_init();             				// Necesaria para que funcione la pantalla
    lcd_spi_init();           				// Inicializa pantalla LCD por SPI
    gfx_init(lcd_draw_pixel, 240, 320);		// Configura librería gráfica (lcd-spi)
}


/* 
 * ----------------------------------------------------------------
 * Funcion displayLCD
 *  - Voltaje de la bateria
 *  - Barra indicadora
 *  - Estado del serial
 *  - Titulo del Giroscopio
 * ----------------------------------------------------------------
*/
void displayLCD(GyroData gyro, float voltage, int sending) {
    // Configuracion inicial de los colores y texto
    char buffer[64];
    gfx_setTextSize(2);
    gfx_fillScreen(LCD_WHITE);
    gfx_setTextColor(LCD_BLACK, LCD_WHITE);

    // 1. Voltaje de la bateria
    gfx_setCursor(10, 10);
    sprintf(buffer, "Bateria: %.2fV", voltage);
    gfx_puts(buffer);

    // 2, Barra indicadora del voltaje
    // 2.1 Inicializacion y declaracion de variables
    int x = 10, y = 35;       // Posición (x, y) inicial donde se dibuja la barra
    int w = 16, h = 12;       // Ancho y alto de cada segmento de la barra
    int gap = 3;              // Espacio entre los segmentos
    int max = 10;             // Número máximo de segmentos a mostrar (10 bloques)
    int fill;                 // Mapea el voltaje a bloques a llenar
    
    // 2.2 Calcula los cuadros a llenar
    fill = (int)((voltage - 2.0f) / 6.5f * max);
    if (fill > max) fill = max;
    if (fill < 0) fill = 0;

    // 2.3 Dibuja el rectangulo y los cuadros
    gfx_drawRect(x - 2, y - 2, max * (w + gap), h + 4, LCD_BLACK);
    for (int i = 0; i < fill; ++i) {
        gfx_fillRect(x + i * (w + gap), y, w, h, LCD_BLACK);
    }

    // 3, Estado del serial
    gfx_setCursor(10, 65);
    gfx_puts("Serial:");
    gfx_setCursor(150, 65);
    gfx_puts(sending ? "ON" : "OFF");


    // 4. Titulo del Giroscopio
    gfx_setCursor(10, 140);
    gfx_puts("Giroscopio");

    // 5. Valores XYZ del giroscopio
    gfx_setCursor(10, 170);
    sprintf(buffer, "x: %d", gyro.x); gfx_puts(buffer);
    gfx_setCursor(10, 200);
    sprintf(buffer, "y: %d", gyro.y); gfx_puts(buffer);
    gfx_setCursor(10, 230);
    sprintf(buffer, "z: %d", gyro.z); gfx_puts(buffer);

    // Muestra en pantalla
    lcd_show_frame();
}



/* 
 * ----------------------------------------------------------------
 * Funcion InitSPI
 *  - Inicializa las variables necesarias para habilitar el uso
 *    de modulo SPI5 al cual está conectado el giroscopio
 * ----------------------------------------------------------------
*/
void init_spi(void) {
    // Habilitación de los relojes
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_GPIOF);
    rcc_periph_clock_enable(RCC_SPI5);

    // Configuracion del CS
    gpio_mode_setup(CS_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, CS_PIN);
    gpio_set(CS_PORT, CS_PIN);

    // Configuración de pines SPI5 (SCK, MISO, MOSI)
    gpio_mode_setup(GPIOF, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO7 | GPIO8 | GPIO9);
    gpio_set_af(GPIOF, GPIO_AF5, GPIO7 | GPIO8 | GPIO9);

    // Configuracion del periferico SPI5
    spi_set_master_mode(SPI5);
    spi_set_baudrate_prescaler(SPI5, SPI_CR1_BR_FPCLK_DIV_64);
    spi_set_clock_polarity_0(SPI5);
    spi_set_clock_phase_0(SPI5);
    spi_set_full_duplex_mode(SPI5);
    spi_set_unidirectional_mode(SPI5);
    spi_enable_software_slave_management(SPI5);
    spi_send_msb_first(SPI5);
    spi_set_nss_high(SPI5);
    SPI_I2SCFGR(SPI5) &= ~SPI_I2SCFGR_I2SMOD;
    spi_enable(SPI5);

    // Inicializacion del giroscopio CTRL1
    gpio_clear(CS_PORT, CS_PIN);
    spi_send(SPI5, GYR_CTRL1);
    spi_read(SPI5);
    spi_send(SPI5, 0x0F | (3 << BW_SHIFT));
    spi_read(SPI5);
    gpio_set(CS_PORT, CS_PIN);

    // Inicializacion del giroscopio CTRL4
    gpio_clear(CS_PORT, CS_PIN);
    spi_send(SPI5, GYR_CTRL4);
    spi_read(SPI5);
    spi_send(SPI5, ((DPS & 0x03) << 5));
    spi_read(SPI5);
    gpio_set(CS_PORT, CS_PIN);
}


/* 
 * ----------------------------------------------------------------
 * Funcion SPI Read
 *  - Genera la secuencia de lectura de registro bajo el protocolo
 *    SPI
 * ----------------------------------------------------------------
*/
uint8_t get_spi_read(uint8_t cmd) {
    gpio_clear(CS_PORT, CS_PIN);
    spi_send(SPI5, cmd);
    spi_read(SPI5);
    spi_send(SPI5, 0);
    uint8_t result = spi_read(SPI5);
    gpio_set(CS_PORT, CS_PIN);
    return result;
}


/* 
 * ----------------------------------------------------------------
 * Funcion ReadGyroAxis
 *  - Realiza la lectura de los registros LOW y HIGH para un eje
 *    del giroscopio, y transforma ambos registros en un solo valor
 *    de 16 bits
 * ----------------------------------------------------------------
*/
int16_t read_gyro_axis(uint8_t lsb, uint8_t msb) {
    uint8_t lo = get_spi_read(lsb | READ_FLAG);
    uint8_t hi = get_spi_read(msb | READ_FLAG);
    return (hi << 8) | lo;
}


/* 
 * ----------------------------------------------------------------
 * Funcion GetGyroData
 *  - Obtiene la lectura de los tres ejes del giroscopio y con la
 *    sensitividad convierte los datos crudos en valores unidades
 *    fisicas de grados por segundo (degrees per second, dps).
 * ----------------------------------------------------------------
*/
GyroData get_gyro_data(void) {
    GyroData vec;
    vec.x = read_gyro_axis(GYR_XL, GYR_XH) * GYR_SENS;
    vec.y = read_gyro_axis(GYR_YL, GYR_YH) * GYR_SENS;
    vec.z = read_gyro_axis(GYR_ZL, GYR_ZH) * GYR_SENS;
    return vec;
}



/* 
 * ----------------------------------------------------------------
 * Funcion InitGPIO
 *  - Habilita los relojes de los puertos GPIO y configura los
 *    pines de los LEDs como salidas y el botón como entrada.
 * ----------------------------------------------------------------
*/
void init_gpio(void) {
    rcc_periph_clock_enable(RCC_GPIOA);     // GPIO bateria y BTN
    rcc_periph_clock_enable(RCC_GPIOG);     // GPIO Leds

    // Configura LEDs como salidas, BTN como entrada
    gpio_mode_setup(LEDS_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GLED_PIN | RLED_PIN);
    gpio_mode_setup(BTN_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, BTN_PIN);
}



/* 
 * ----------------------------------------------------------------
 * Funcion InitADC
 *  - Habilita el reloj del ADC1, configura el pin como entrada
 *    analógica y ajusta los parámetros básicos de conversión.
 * ----------------------------------------------------------------
*/
void init_adc(void) {
    rcc_periph_clock_enable(RCC_ADC1);
    gpio_mode_setup(BAT_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, BAT_PIN);
    adc_power_off(ADC1);
    adc_disable_scan_mode(ADC1);
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_3CYC);
    adc_power_on(ADC1);
}


/* 
 * ----------------------------------------------------------------
 * Funcion ReadVoltage
 *  - Realiza una conversión ADC en el canal indicado y devuelve
 *    el voltaje convertido a valor real en voltios.
 * ----------------------------------------------------------------
*/
float read_voltage(uint8_t channel) {
    uint8_t seq[1] = {channel};
    adc_set_regular_sequence(ADC1, 1, seq);
    adc_start_conversion_regular(ADC1);
    while (!adc_eoc(ADC1));
    return adc_read_regular(ADC1) * (MAX_VOLT / 4095.0f);
}



/* 
 * ----------------------------------------------------------------
 * Funcion StatusLeds
 *  - Enciende o apaga los LEDs de estado según si se está
 *    transmitiendo y si el voltaje es menor a 7.0V.
 * ----------------------------------------------------------------
*/
void status_leds(int sending, float voltage) {
    if (sending) {
        gpio_set(LEDS_PORT, GLED_PIN);
    } else {
        gpio_clear(LEDS_PORT, GLED_PIN);
    }

    if (voltage < 7.0f) {
        gpio_set(LEDS_PORT, RLED_PIN);
    } else {
        gpio_clear(LEDS_PORT, RLED_PIN);
    }
}



/* 
 * ----------------------------------------------------------------
 * Funcion InitUSART
 *  - Configura el USART1 para transmisión serial a 115200 baudios
 *    utilizando el pin configurado como salida alternativa.
 * ----------------------------------------------------------------
*/
void init_usart(void) {
    gpio_mode_setup(USART1_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, USART1_TX);
    gpio_set_af(USART1_PORT, GPIO_AF7, USART1_TX);
    
    rcc_periph_clock_enable(RCC_USART1);
    usart_set_baudrate(USART1, 115200);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_set_mode(USART1, USART_MODE_TX);
    usart_enable(USART1);
}



/* 
 * ----------------------------------------------------------------
 * Funcion TransmitUSART
 *  - Transmite por USART los datos del giroscopio y voltaje en
 *    formato tabulado y legible por consola.
 * ----------------------------------------------------------------
*/
void xmit_usart(GyroData g, float v) {
    char buffer[64];
    int len = snprintf(buffer, sizeof(buffer), "%d\t%d\t%d\t%.2f\n", g.x, g.y, g.z, v);
    for (int i = 0; i < len; ++i) {
        usart_send_blocking(USART1, buffer[i]);
    }
}


/* 
 * ----------------------------------------------------------------
 * Funcion Setup
 *  - Configura el estado inicial del sistema (LCD, giroscopio, 
 *    bateria, LEDs, botón y USART)
 * ----------------------------------------------------------------
*/
void setup(void) {
    init_gpio();
    init_usart();       // Va luego de gpio por el RCC_GPIOA
    init_adc();
    init_spi();         // Este debe ir primero que init_lcd
    init_lcd();         // de lo contrario no funciona
}




/* 
 * ----------------------------------------------------------------
 * Funcion Main
 *  - Ejecucion ciclica
 *  - Instancia las funciones anteriores y controla el flujo de
 *    ejecución
 * ----------------------------------------------------------------
*/
int main(void) {
    setup();

    bool serial = false;
    bool prev_btn = false;


    while(1) {
        // Calcular los valores
        GyroData gdata = get_gyro_data();
        float vltg = read_voltage(CHANNEL);
        
        bool btn = gpio_get(BTN_PORT, BTN_PIN);
        if (btn && !prev_btn) serial = !serial;
        prev_btn = btn;

        // Mostrar resultados
        displayLCD(gdata, vltg, serial);
        status_leds(serial, vltg);

        // Enviar datos por serial
        if (serial) xmit_usart(gdata, vltg);
        for (int i = 0; i < 120000; ++i) __asm__("nop");
    }
}
