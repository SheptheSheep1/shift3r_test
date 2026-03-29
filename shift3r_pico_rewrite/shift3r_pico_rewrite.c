#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"

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

// Constants
#define SERVO_FRAME_US    20000u   // 20 ms frame => 50 Hz
#define SERVO_MIN_US      1000u    // 1.0 ms (min throttle)
#define SERVO_NEUTRAL_US  1500u    // 1.5 ms (neutral throttle)
#define SERVO_MAX_US      2000u    // 2.0 ms (max throttle)
#define PWM_CLOCK_DIV     125.0f   // 125 MHz / 125 = 1 MHz tick (1 us per tick)
#define RAMP_STEP_US      5u       // microseconds per ramp step
#define RAMP_STEP_DELAY_MS 1       // ms delay between ramp steps for smoothness

int32_t enc_count = 0;
int32_t new_enc_count = 0;

enum ShiftingState {IDLE, SHIFTING_UP, SHIFTING_DOWN, ERROR};

// Initialize a GPIO for servo-style PWM (1 µs resolution, 50 Hz)
static void servo_pwm_init_gpio(uint gpio) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);

    uint slice = pwm_gpio_to_slice_num(gpio);

    // Make PWM tick = 1 MHz (1 µs)
    float clk_sys_hz = (float)clock_get_hz(clk_sys);
    float clkdiv = clk_sys_hz / 1000000.0f;
    pwm_set_clkdiv(slice, clkdiv);

    // 20 ms period
    pwm_set_wrap(slice, SERVO_FRAME_US - 1u);

    // Set initial level to minimum safe pulse
    uint channel = pwm_gpio_to_channel(gpio);
    if (channel == 0) {
        pwm_set_chan_level(slice, PWM_CHAN_A, SERVO_MIN_US);
    } else {
        pwm_set_chan_level(slice, PWM_CHAN_B, SERVO_MIN_US);
    }

    pwm_set_enabled(slice, true);
}

// Set servo pulse width in microseconds
static void servo_set_pulse_us(uint gpio, uint32_t pulse_us) {
    if (pulse_us < SERVO_MIN_US) pulse_us = SERVO_MIN_US;
    if (pulse_us > SERVO_MAX_US) pulse_us = SERVO_MAX_US;

    uint slice = pwm_gpio_to_slice_num(gpio);
    uint channel = pwm_gpio_to_channel(gpio);

    if (channel == 0) {
        pwm_set_chan_level(slice, PWM_CHAN_A, pulse_us);
    } else {
        pwm_set_chan_level(slice, PWM_CHAN_B, pulse_us);
    }
}

int main()
{
    stdio_init_all();
    enum ShiftingState shiftState = IDLE;
    PIO quadrature_pio = pio0;
    const uint quad_sm = 0;
    pio_add_program(quadrature_pio, &quadrature_encoder_program);
    quadrature_encoder_program_init(quadrature_pio, quad_sm, ENCODER_PIN_A, 0);
    servo_pwm_init_gpio(ESC_PIN_PWM);


    while (true) {
        new_enc_count = quadrature_encoder_get_count(quadrature_pio, quad_sm);
        if(enc_count != new_enc_count){enc_count = new_enc_count; printf("\nenc: %8d", enc_count);}
        switch (shiftState)
        {
        case IDLE:
            /* code */
            break;
        case SHIFTING_DOWN:
        case SHIFTING_UP:
        
        default:
            break;
        }
    
        //printf("Hello, world!\n");
        //printf("enc: %8d", enc_count);
        //sleep_ms(1000);
                // Mid throttle (1.5 ms)
        servo_set_pulse_us(ESC_PIN_PWM, 1500);
        sleep_ms(2000);

        // Back to idle
        servo_set_pulse_us(ESC_PIN_PWM, SERVO_MIN_US);
        sleep_ms(2000);
    }
}
