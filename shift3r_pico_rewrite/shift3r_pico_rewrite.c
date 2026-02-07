#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "hardware/pio.h"

#include "quadrature_encoder.pio.h"

#define ENCODER_PIN_A 3
#define ENCODER_PIN_B 4
#define LED_1 5
#define LED_2 6
#define SHIFT_DOWN_PIN 28
#define ESC_PIN_PWM 26
#define SHIFT_UP_PIN 27
#define UP 1
#define DOWN 0
#define ECU_PWM_PIN 2
#define ECU_RANGE 255
#define TIME_UP_PIN 1
#define SHIFT_TIMEOUT_MS 500

int32_t enc_count = 0;
int32_t new_enc_count = 0;


int main()
{
    stdio_init_all();
    PIO quadrature_pio = pio0;
    const uint sm = 0;
    pio_add_program(quadrature_pio, &quadrature_encoder_program);
    quadrature_encoder_program_init(quadrature_pio, sm, ENCODER_PIN_A, 0);



    while (true) {
        new_enc_count = quadrature_encoder_get_count(quadrature_pio, sm);
        if(enc_count != new_enc_count){enc_count = new_enc_count; printf("\nenc: %8d", enc_count);}
        //printf("Hello, world!\n");
        //printf("enc: %8d", enc_count);
        //sleep_ms(1000);
    }
}
