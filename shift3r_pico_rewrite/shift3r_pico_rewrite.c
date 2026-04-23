#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/flash.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"

#include "quadrature_encoder.pio.h"
#include "ws2812.pio.h"

#define FLASH_TARGET_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)

#ifndef ENABLE_GEAR_PERSISTENCE
#define ENABLE_GEAR_PERSISTENCE 1
#endif

#ifndef ENABLE_ESC_RAMP
#define ENABLE_ESC_RAMP 0
#endif

#ifndef ENABLE_ESC_P_CONTROL
#define ENABLE_ESC_P_CONTROL 0
#endif

#define ENCODER_PIN_A 3
#define ENCODER_PIN_B 4
#define SHIFT_DOWN_PIN 8
#define SHIFT_UP_PIN 6
#define ECU_PWM_PIN 2
#define ESC_PIN_PWM 17
#define RGB_DATA_PIN 22
#define RGB_POWER_PIN 23

#define SHIFT_TIMEOUT_US 250000ULL
#define SHIFT_PEAK_HOLD_US 10000ULL
#define RETURN_HOME_TIMEOUT_US 200000ULL
#define ESC_ACTIVE_LIMIT_US 275000ULL
#define FAULT_COOLDOWN_US 300000ULL
#define BUTTON_DEBOUNCE_US 0ULL
#define SHIFT_REQUEST_MIN_ACTIVE_US 500ULL
#define STATUS_LED_UPDATE_US 20000ULL
#define LOOP_SLEEP_US 100ULL
#define MAIN_LOOP_WATCHDOG_MS 1000

#define FULL_SHIFT_COUNTS 290
#define HALF_SHIFT_COUNTS 180
#define SHIFT_TOLERANCE_COUNTS 32

#define UP_COUNT_SIGN (-1)
#define DOWN_COUNT_SIGN (1)

#define ESC_PWM_PERIOD_US 20000U
#define ESC_PWM_NEUTRAL_US 1500U
#define ESC_PWM_SHIFT_UP_US 2000U
#define ESC_PWM_SHIFT_DOWN_US 1000U
#define ESC_PWM_HOLD_UP_US ESC_PWM_SHIFT_UP_US
#define ESC_PWM_HOLD_DOWN_US ESC_PWM_SHIFT_DOWN_US
#define ESC_RAMP_DURATION_US 20000ULL
#define ESC_P_FULL_US_PER_COUNT 0.75f
#define ESC_P_HALF_US_PER_COUNT 1.25f
#define ESC_P_FULL_BIAS_US 0U
#define ESC_P_HALF_BIAS_US 0U
#define ESC_P_MAX_DELTA_US (ESC_PWM_NEUTRAL_US - ESC_PWM_SHIFT_UP_US)

#define ECU_PWM_TOP 255U
#define ECU_PWM_FREQ_HZ 100000U
#define ECU_DUTY_NEUTRAL 19U
#define ECU_DUTY_GEAR_1 59U
#define ECU_DUTY_GEAR_2 98U
#define ECU_DUTY_GEAR_3 138U
#define ECU_DUTY_GEAR_4 177U
#define ECU_DUTY_GEAR_5 216U
#define ECU_DUTY_GEAR_6 255U

#define STATE_TASK_BUDGET_US 1000ULL
#define LED_TASK_BUDGET_US 1000ULL
#define PERSIST_TASK_BUDGET_US 25000ULL

typedef enum {
    SHIFT_STATE_IDLE = 0,
    SHIFT_STATE_SHIFT_UP,
    SHIFT_STATE_SHIFT_DOWN,
    SHIFT_STATE_PEAK_HOLD,
    SHIFT_STATE_RETURN_HOME,
    SHIFT_STATE_FAULT_COOLDOWN,
} shift_state_t;

typedef enum {
    GEAR_NEUTRAL = 0,
    GEAR_1,
    GEAR_2,
    GEAR_3,
    GEAR_4,
    GEAR_5,
    GEAR_6,
    GEAR_COUNT
} gear_t;

typedef enum {
    SHIFT_DIR_UP = 0,
    SHIFT_DIR_DOWN
} shift_direction_t;

typedef struct {
    uint gpio;
    uint slice;
    uint channel;
    uint32_t wrap;
} pwm_output_t;

typedef struct {
    uint gpio;
    volatile bool candidate_active;
    volatile bool validated_active;
    volatile uint32_t candidate_started_us;
} shift_input_t;

typedef struct {
    shift_state_t state;
    gear_t gear;
    gear_t target_gear;
    shift_direction_t direction;
    int32_t shift_origin_count;
    int32_t last_encoder_count;
    uint32_t target_counts;
    int8_t count_sign;
    uint64_t state_started_us;
    uint64_t shift_started_us;
    uint64_t fault_started_us;
    uint64_t last_status_led_us;
    uint64_t esc_active_since_us;
    uint64_t esc_ramp_started_us;
    uint16_t esc_target_us;
    uint16_t esc_current_us;
    uint16_t esc_ramp_start_us;
    const char *fault_reason;
    bool uses_half_shift;
    bool esc_is_active;
} controller_t;

static shift_input_t g_shift_up_input = {.gpio = SHIFT_UP_PIN};
static shift_input_t g_shift_down_input = {.gpio = SHIFT_DOWN_PIN};

static const uint8_t k_ecu_duty_by_gear[GEAR_COUNT] = {
    ECU_DUTY_NEUTRAL,
    ECU_DUTY_GEAR_1,
    ECU_DUTY_GEAR_2,
    ECU_DUTY_GEAR_3,
    ECU_DUTY_GEAR_4,
    ECU_DUTY_GEAR_5,
    ECU_DUTY_GEAR_6,
};

static PIO g_encoder_pio = pio0;
static uint g_encoder_sm = 0;
static PIO g_ws2812_pio = pio1;
static uint g_ws2812_sm = 0;

static pwm_output_t g_esc_pwm;
static pwm_output_t g_ecu_pwm;
static controller_t g_controller;

static inline uint64_t now_us(void) {
    return time_us_64();
}

static inline uint32_t clamp_u32(uint32_t value, uint32_t low, uint32_t high) {
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)g << 16) | ((uint32_t)r << 8) | (uint32_t)b;
}

static inline const char *gear_name(gear_t gear) {
    static const char *const k_names[GEAR_COUNT] = {"N", "1", "2", "3", "4", "5", "6"};
    return (gear < GEAR_COUNT) ? k_names[gear] : "?";
}

static void pwm_output_init(pwm_output_t *output, uint gpio, uint32_t wrap, float divider) {
    output->gpio = gpio;
    output->slice = pwm_gpio_to_slice_num(gpio);
    output->channel = pwm_gpio_to_channel(gpio);
    output->wrap = wrap;

    gpio_set_function(gpio, GPIO_FUNC_PWM);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, wrap);
    pwm_config_set_clkdiv(&config, divider);
    pwm_init(output->slice, &config, true);
    pwm_set_chan_level(output->slice, output->channel, 0);
}

static void pwm_output_set_level(const pwm_output_t *output, uint32_t level) {
    pwm_set_chan_level(output->slice, output->channel, clamp_u32(level, 0, output->wrap));
}

static void ws2812_put_pixel(uint32_t grb) {
    pio_sm_put_blocking(g_ws2812_pio, g_ws2812_sm, grb << 8u);
}

static void update_status_led(const controller_t *controller) {
    uint32_t color = 0;

    switch (controller->state) {
        case SHIFT_STATE_IDLE:
            color = ENABLE_GEAR_PERSISTENCE ? urgb_u32(0, 24, 0) : urgb_u32(0, 10, 24);
            break;
        case SHIFT_STATE_SHIFT_UP:
            color = urgb_u32(0, 0, 24);
            break;
        case SHIFT_STATE_SHIFT_DOWN:
            color = urgb_u32(24, 16, 0);
            break;
        case SHIFT_STATE_PEAK_HOLD:
            color = urgb_u32(24, 24, 0);
            break;
        case SHIFT_STATE_RETURN_HOME:
            color = urgb_u32(0, 24, 24);
            break;
        case SHIFT_STATE_FAULT_COOLDOWN:
            color = urgb_u32(24, 0, 0);
            break;
    }

    ws2812_put_pixel(color);
}

static void esc_apply_command(controller_t *controller, uint64_t timestamp_us) {
#if ENABLE_ESC_RAMP
    if (controller->esc_current_us != controller->esc_target_us) {
        const uint64_t elapsed = timestamp_us - controller->esc_ramp_started_us;
        if (elapsed >= ESC_RAMP_DURATION_US) {
            controller->esc_current_us = controller->esc_target_us;
        } else {
            const int32_t delta = (int32_t)controller->esc_target_us - (int32_t)controller->esc_ramp_start_us;
            controller->esc_current_us =
                (uint16_t)(controller->esc_ramp_start_us + ((delta * (int32_t)elapsed) / (int32_t)ESC_RAMP_DURATION_US));
        }
    }
#else
    (void)timestamp_us;
    controller->esc_current_us = controller->esc_target_us;
#endif

    pwm_output_set_level(&g_esc_pwm, controller->esc_current_us);

    if (controller->esc_current_us == ESC_PWM_NEUTRAL_US) {
        controller->esc_is_active = false;
        controller->esc_active_since_us = 0;
    } else if (!controller->esc_is_active) {
        controller->esc_is_active = true;
        controller->esc_active_since_us = timestamp_us;
    }
}

static void esc_set_target(controller_t *controller, uint16_t pulse_us, uint64_t timestamp_us) {
    controller->esc_target_us = pulse_us;
#if ENABLE_ESC_RAMP
    controller->esc_ramp_start_us = controller->esc_current_us;
    controller->esc_ramp_started_us = timestamp_us;
#else
    controller->esc_current_us = pulse_us;
#endif
    esc_apply_command(controller, timestamp_us);
}

static void write_ecu_gear_output(gear_t gear) {
    if (gear >= GEAR_COUNT) {
        gear = GEAR_NEUTRAL;
    }
    pwm_output_set_level(&g_ecu_pwm, k_ecu_duty_by_gear[gear]);
}

static bool is_adjacent_to_neutral(gear_t a, gear_t b) {
    return ((a == GEAR_1 || a == GEAR_2) && b == GEAR_NEUTRAL) ||
           (a == GEAR_NEUTRAL && (b == GEAR_1 || b == GEAR_2));
}

static gear_t next_gear_for_request(gear_t current, shift_direction_t direction) {
    if (direction == SHIFT_DIR_UP) {
        switch (current) {
            case GEAR_1:
                return GEAR_NEUTRAL;
            case GEAR_NEUTRAL:
                return GEAR_2;
            case GEAR_2:
                return GEAR_3;
            case GEAR_3:
                return GEAR_4;
            case GEAR_4:
                return GEAR_5;
            case GEAR_5:
                return GEAR_6;
            default:
                return GEAR_COUNT;
        }
    }

    switch (current) {
        case GEAR_6:
            return GEAR_5;
        case GEAR_5:
            return GEAR_4;
        case GEAR_4:
            return GEAR_3;
        case GEAR_3:
            return GEAR_2;
        case GEAR_2:
            return GEAR_NEUTRAL;
        case GEAR_NEUTRAL:
            return GEAR_1;
        default:
            return GEAR_COUNT;
    }
}

static uint32_t shift_counts_for_transition(gear_t current, gear_t target) {
    return is_adjacent_to_neutral(current, target) ? HALF_SHIFT_COUNTS : FULL_SHIFT_COUNTS;
}

static uint16_t fixed_shift_pulse_for_direction(shift_direction_t direction) {
    return (direction == SHIFT_DIR_UP) ? ESC_PWM_SHIFT_UP_US : ESC_PWM_SHIFT_DOWN_US;
}

static uint16_t hold_pulse_for_direction(shift_direction_t direction) {
    return (direction == SHIFT_DIR_UP) ? ESC_PWM_HOLD_UP_US : ESC_PWM_HOLD_DOWN_US;
}

static uint32_t remaining_counts_from_progress(int32_t signed_progress, uint32_t target_counts) {
    int32_t remaining_counts = (int32_t)target_counts - signed_progress;
    if (remaining_counts < 0) {
        remaining_counts = 0;
    }
    return (uint32_t)remaining_counts;
}

static uint16_t compute_p_control_pulse(shift_direction_t direction, uint32_t remaining_counts, bool uses_half_shift) {
    const float gain = uses_half_shift ? ESC_P_HALF_US_PER_COUNT : ESC_P_FULL_US_PER_COUNT;
    const uint32_t bias = uses_half_shift ? ESC_P_HALF_BIAS_US : ESC_P_FULL_BIAS_US;
    const uint32_t pulse_delta = clamp_u32(bias + (uint32_t)(gain * (float)remaining_counts), 0, ESC_P_MAX_DELTA_US);

    if (direction == SHIFT_DIR_UP) {
        return (uint16_t)(ESC_PWM_NEUTRAL_US - pulse_delta);
    }
    return (uint16_t)(ESC_PWM_NEUTRAL_US + pulse_delta);
}

static int8_t count_sign_for_direction(shift_direction_t direction) {
    return (direction == SHIFT_DIR_UP) ? UP_COUNT_SIGN : DOWN_COUNT_SIGN;
}

static bool progress_reached_target(int32_t raw_delta, int8_t sign, uint32_t target_counts) {
    const int32_t signed_progress = raw_delta * sign;
    const int32_t threshold = (int32_t)target_counts - (int32_t)SHIFT_TOLERANCE_COUNTS;
    return signed_progress >= threshold;
}

static void drive_direction(controller_t *controller,
                            shift_direction_t direction,
                            uint32_t remaining_counts,
                            bool uses_half_shift,
                            uint64_t timestamp_us) {
#if ENABLE_ESC_P_CONTROL
    controller->esc_target_us = compute_p_control_pulse(direction, remaining_counts, uses_half_shift);
    controller->esc_current_us = controller->esc_target_us;
    controller->esc_ramp_start_us = controller->esc_target_us;
    controller->esc_ramp_started_us = timestamp_us;
    esc_apply_command(controller, timestamp_us);
#else
    (void)remaining_counts;
    (void)uses_half_shift;
    esc_set_target(controller, fixed_shift_pulse_for_direction(direction), timestamp_us);
#endif
}

static void enter_fault_cooldown(controller_t *controller, const char *reason, uint64_t timestamp_us) {
    controller->state = SHIFT_STATE_FAULT_COOLDOWN;
    controller->fault_reason = reason;
    controller->fault_started_us = timestamp_us;
    controller->state_started_us = timestamp_us;
    controller->target_counts = 0;
    controller->target_gear = controller->gear;
    esc_set_target(controller, ESC_PWM_NEUTRAL_US, timestamp_us);
    printf("FAULT: %s\n", reason);
}

static bool persist_gear(gear_t gear) {
#if ENABLE_GEAR_PERSISTENCE
    uint8_t buffer[FLASH_PAGE_SIZE];
    memset(buffer, 0xff, sizeof(buffer));
    buffer[0] = (uint8_t)gear;

    int first_empty_page = -1;
    for (uint page = 0; page < FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE; ++page) {
        const uint32_t address = XIP_BASE + FLASH_TARGET_OFFSET + (page * FLASH_PAGE_SIZE);
        const uint8_t value = *((const uint8_t *)address);
        if (value == 0xffu) {
            first_empty_page = (int)page;
            break;
        }
    }

    if (first_empty_page < 0) {
        const uint32_t interrupt_state = save_and_disable_interrupts();
        flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
        restore_interrupts(interrupt_state);
        first_empty_page = 0;
    }

    const uint32_t interrupt_state = save_and_disable_interrupts();
    flash_range_program(FLASH_TARGET_OFFSET + ((uint32_t)first_empty_page * FLASH_PAGE_SIZE), buffer, FLASH_PAGE_SIZE);
    restore_interrupts(interrupt_state);
    return true;
#else
    (void)gear;
    return false;
#endif
}

static gear_t restore_gear(void) {
#if ENABLE_GEAR_PERSISTENCE
    gear_t last_valid = GEAR_NEUTRAL;
    bool found = false;

    for (uint page = 0; page < FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE; ++page) {
        const uint32_t address = XIP_BASE + FLASH_TARGET_OFFSET + (page * FLASH_PAGE_SIZE);
        const uint8_t value = *((const uint8_t *)address);
        if (value == 0xffu) {
            break;
        }
        if (value < GEAR_COUNT) {
            last_valid = (gear_t)value;
            found = true;
        }
    }

    if (found) {
        printf("Restored gear from flash: %s\n", gear_name(last_valid));
        return last_valid;
    }
#endif
    printf("No persisted gear, starting in N\n");
    return GEAR_NEUTRAL;
}

static void begin_shift(controller_t *controller, shift_direction_t direction, int32_t encoder_count, uint64_t timestamp_us) {
    const gear_t target = next_gear_for_request(controller->gear, direction);
    if (target >= GEAR_COUNT) {
        printf("Ignoring %s request in gear %s\n",
               direction == SHIFT_DIR_UP ? "up" : "down",
               gear_name(controller->gear));
        return;
    }

    controller->direction = direction;
    controller->target_gear = target;
    controller->target_counts = shift_counts_for_transition(controller->gear, target);
    controller->uses_half_shift = is_adjacent_to_neutral(controller->gear, target);
    controller->count_sign = count_sign_for_direction(direction);
    controller->shift_origin_count = encoder_count;
    controller->state_started_us = timestamp_us;
    controller->shift_started_us = timestamp_us;
    controller->state = (direction == SHIFT_DIR_UP) ? SHIFT_STATE_SHIFT_UP : SHIFT_STATE_SHIFT_DOWN;

    drive_direction(controller, controller->direction, controller->target_counts, controller->uses_half_shift, timestamp_us);

    printf("Shift %s: %s -> %s, target counts=%lu, mode=%s\n",
           direction == SHIFT_DIR_UP ? "up" : "down",
           gear_name(controller->gear),
           gear_name(controller->target_gear),
           (unsigned long)controller->target_counts,
           controller->uses_half_shift ? "half" : "full");
}

static void finish_shift_cycle(controller_t *controller, uint64_t timestamp_us, uint64_t return_elapsed_us) {
    controller->state = SHIFT_STATE_IDLE;
    controller->state_started_us = timestamp_us;
    controller->target_counts = 0;
    esc_set_target(controller, ESC_PWM_NEUTRAL_US, timestamp_us);

    printf("Return home complete, elapsed=%llu us (%.2f ms)\n",
           (unsigned long long)return_elapsed_us,
           (double)return_elapsed_us / 1000.0);

#if ENABLE_GEAR_PERSISTENCE
    const uint64_t persist_started_us = now_us();
    if (!persist_gear(controller->gear)) {
        enter_fault_cooldown(controller, "flash persist failed", timestamp_us);
        return;
    }
    if ((now_us() - persist_started_us) > PERSIST_TASK_BUDGET_US) {
        enter_fault_cooldown(controller, "flash persist runtime exceeded", now_us());
        return;
    }
#endif
}

static void cancel_active_shift(controller_t *controller, const char *reason, uint64_t timestamp_us) {
    controller->state = SHIFT_STATE_IDLE;
    controller->state_started_us = timestamp_us;
    controller->target_counts = 0;
    controller->target_gear = controller->gear;
    esc_set_target(controller, ESC_PWM_NEUTRAL_US, timestamp_us);

    printf("%s\n", reason);
}

static void start_return_home(controller_t *controller, int32_t encoder_count, uint64_t timestamp_us) {
    const uint64_t shift_elapsed_us = timestamp_us - controller->shift_started_us;
    controller->gear = controller->target_gear;
    write_ecu_gear_output(controller->gear);

    printf("Shift complete: now in gear %s, elapsed=%llu us (%.2f ms), returning home\n",
           gear_name(controller->gear),
           (unsigned long long)shift_elapsed_us,
           (double)shift_elapsed_us / 1000.0);

    const int32_t home_error = controller->shift_origin_count - encoder_count;
    const uint32_t remaining_counts = (uint32_t)((home_error < 0) ? -home_error : home_error);
    if (remaining_counts <= SHIFT_TOLERANCE_COUNTS) {
        finish_shift_cycle(controller, timestamp_us, 0);
        return;
    }

    controller->state = SHIFT_STATE_RETURN_HOME;
    controller->state_started_us = timestamp_us;

    drive_direction(controller,
                    (home_error > 0) ? SHIFT_DIR_DOWN : SHIFT_DIR_UP,
                    remaining_counts,
                    controller->uses_half_shift,
                    timestamp_us);
}

static void enter_peak_hold(controller_t *controller, uint64_t timestamp_us) {
    controller->state = SHIFT_STATE_PEAK_HOLD;
    controller->state_started_us = timestamp_us;
    esc_set_target(controller, hold_pulse_for_direction(controller->direction), timestamp_us);

    printf("Peak reached, holding for %llu us (%.2f ms)\n",
           (unsigned long long)SHIFT_PEAK_HOLD_US,
           (double)SHIFT_PEAK_HOLD_US / 1000.0);
}

static void handle_shift_state(controller_t *controller, int32_t encoder_count, uint64_t timestamp_us) {
    const int32_t raw_delta = encoder_count - controller->shift_origin_count;
    const int32_t signed_progress = raw_delta * controller->count_sign;
    const uint32_t remaining_counts = remaining_counts_from_progress(signed_progress, controller->target_counts);

    if (progress_reached_target(raw_delta, controller->count_sign, controller->target_counts)) {
        enter_peak_hold(controller, timestamp_us);
        return;
    }

    drive_direction(controller, controller->direction, remaining_counts, controller->uses_half_shift, timestamp_us);

    if ((timestamp_us - controller->state_started_us) > SHIFT_TIMEOUT_US) {
        const uint64_t shift_elapsed_us = timestamp_us - controller->state_started_us;
        printf("Shift timeout after %llu us (%.2f ms), progress=%ld / %lu counts\n",
               (unsigned long long)shift_elapsed_us,
               (double)shift_elapsed_us / 1000.0,
               (long)(raw_delta * controller->count_sign),
               (unsigned long)controller->target_counts);
        enter_fault_cooldown(controller, "shift timeout", timestamp_us);
        return;
    }
}

static void handle_peak_hold_state(controller_t *controller, int32_t encoder_count, uint64_t timestamp_us) {
    esc_set_target(controller, hold_pulse_for_direction(controller->direction), timestamp_us);

    if ((timestamp_us - controller->state_started_us) >= SHIFT_PEAK_HOLD_US) {
        start_return_home(controller, encoder_count, timestamp_us);
    }
}

static void handle_return_home_state(controller_t *controller, int32_t encoder_count, uint64_t timestamp_us) {
    const int32_t home_error = controller->shift_origin_count - encoder_count;
    const uint32_t remaining_counts = (uint32_t)((home_error < 0) ? -home_error : home_error);

    if (remaining_counts <= SHIFT_TOLERANCE_COUNTS) {
        finish_shift_cycle(controller, timestamp_us, timestamp_us - controller->state_started_us);
        return;
    }

    drive_direction(controller,
                    (home_error > 0) ? SHIFT_DIR_DOWN : SHIFT_DIR_UP,
                    remaining_counts,
                    controller->uses_half_shift,
                    timestamp_us);

    if ((timestamp_us - controller->state_started_us) > RETURN_HOME_TIMEOUT_US) {
        const uint64_t return_elapsed_us = timestamp_us - controller->state_started_us;
        printf("Return home timeout after %llu us (%.2f ms), remaining=%lu counts\n",
               (unsigned long long)return_elapsed_us,
               (double)return_elapsed_us / 1000.0,
               (unsigned long)remaining_counts);
        enter_fault_cooldown(controller, "return home timeout", timestamp_us);
    }
}

static void service_state_machine(controller_t *controller, int32_t encoder_count, uint64_t timestamp_us) {
    const bool shift_up_active = g_shift_up_input.validated_active;
    const bool shift_down_active = g_shift_down_input.validated_active;

    controller->last_encoder_count = encoder_count;
    esc_apply_command(controller, timestamp_us);

    if (controller->esc_is_active && (timestamp_us - controller->esc_active_since_us) > ESC_ACTIVE_LIMIT_US) {
        enter_fault_cooldown(controller, "ESC active limit exceeded", timestamp_us);
        return;
    }

    switch (controller->state) {
        case SHIFT_STATE_IDLE:
            if (shift_up_active && shift_down_active) {
                enter_fault_cooldown(controller, "conflicting shift requests", timestamp_us);
                return;
            }

            if (shift_up_active) {
                begin_shift(controller, SHIFT_DIR_UP, encoder_count, timestamp_us);
            } else if (shift_down_active) {
                begin_shift(controller, SHIFT_DIR_DOWN, encoder_count, timestamp_us);
            }
            break;

        case SHIFT_STATE_SHIFT_UP:
            if (shift_down_active) {
                enter_fault_cooldown(controller, "conflicting shift requests", timestamp_us);
                return;
            }
            if (!shift_up_active) {
                cancel_active_shift(controller, "Shift up command released, stopping actuation", timestamp_us);
                return;
            }
            handle_shift_state(controller, encoder_count, timestamp_us);
            break;

        case SHIFT_STATE_SHIFT_DOWN:
            if (shift_up_active) {
                enter_fault_cooldown(controller, "conflicting shift requests", timestamp_us);
                return;
            }
            if (!shift_down_active) {
                cancel_active_shift(controller, "Shift down command released, stopping actuation", timestamp_us);
                return;
            }
            handle_shift_state(controller, encoder_count, timestamp_us);
            break;

        case SHIFT_STATE_PEAK_HOLD:
            if (shift_up_active && shift_down_active) {
                enter_fault_cooldown(controller, "conflicting shift requests", timestamp_us);
                return;
            }
            if ((controller->direction == SHIFT_DIR_UP && !shift_up_active) ||
                (controller->direction == SHIFT_DIR_DOWN && !shift_down_active)) {
                cancel_active_shift(controller, "Shift command released during hold, stopping actuation", timestamp_us);
                return;
            }
            handle_peak_hold_state(controller, encoder_count, timestamp_us);
            break;

        case SHIFT_STATE_RETURN_HOME:
            if (shift_up_active && shift_down_active) {
                enter_fault_cooldown(controller, "conflicting shift requests", timestamp_us);
                return;
            }
            if ((controller->direction == SHIFT_DIR_UP && !shift_up_active) ||
                (controller->direction == SHIFT_DIR_DOWN && !shift_down_active)) {
                cancel_active_shift(controller, "Shift command released during return-home, stopping actuation", timestamp_us);
                return;
            }
            handle_return_home_state(controller, encoder_count, timestamp_us);
            break;

        case SHIFT_STATE_FAULT_COOLDOWN:
            if ((timestamp_us - controller->fault_started_us) >= FAULT_COOLDOWN_US) {
                controller->fault_reason = NULL;
                controller->state = SHIFT_STATE_IDLE;
                controller->state_started_us = timestamp_us;
                esc_set_target(controller, ESC_PWM_NEUTRAL_US, timestamp_us);
                printf("Fault cooldown complete, returning to idle\n");
            }
            break;
    }
}

static void enforce_task_budget(controller_t *controller, const char *task_name, uint64_t started_us, uint64_t budget_us) {
    const uint64_t elapsed = now_us() - started_us;
    if (elapsed > budget_us) {
        enter_fault_cooldown(controller, task_name, now_us());
    }
}

static void service_shift_input(shift_input_t *input, uint32_t timestamp_us) {
    if (gpio_get(input->gpio) != 0) {
        input->candidate_active = false;
        input->validated_active = false;
        return;
    }

    if (!input->candidate_active) {
        input->candidate_active = true;
        input->candidate_started_us = timestamp_us;
        return;
    }

    if ((uint32_t)(timestamp_us - input->candidate_started_us) < SHIFT_REQUEST_MIN_ACTIVE_US) {
        return;
    }

    input->validated_active = true;
}

static void service_shift_requests(uint64_t timestamp_us) {
    const uint32_t timestamp32_us = (uint32_t)timestamp_us;
    service_shift_input(&g_shift_up_input, timestamp32_us);
    service_shift_input(&g_shift_down_input, timestamp32_us);
}

static void init_encoder_pio(void) {
    const uint offset = pio_add_program(g_encoder_pio, &quadrature_encoder_program);
    if (offset != 0u) {
        printf("Quadrature program offset mismatch: %u\n", offset);
    }
    quadrature_encoder_program_init(g_encoder_pio, g_encoder_sm, ENCODER_PIN_A, 0);
}

static void init_ws2812_pio(void) {
    const uint offset = pio_add_program(g_ws2812_pio, &ws2812_program);
    ws2812_program_init(g_ws2812_pio, g_ws2812_sm, offset, RGB_DATA_PIN, 800000.0f, false);
}

static void init_shift_inputs(void) {
    gpio_init(SHIFT_UP_PIN);
    gpio_set_dir(SHIFT_UP_PIN, GPIO_IN);
    gpio_pull_up(SHIFT_UP_PIN);

    gpio_init(SHIFT_DOWN_PIN);
    gpio_set_dir(SHIFT_DOWN_PIN, GPIO_IN);
    gpio_pull_up(SHIFT_DOWN_PIN);
}

static void init_rgb_power(void) {
    gpio_init(RGB_POWER_PIN);
    gpio_set_dir(RGB_POWER_PIN, GPIO_OUT);
    gpio_put(RGB_POWER_PIN, 1);
}

static void init_pwm_outputs(void) {
    const float esc_divider = (float)clock_get_hz(clk_sys) / 1000000.0f;
    const float ecu_divider = (float)clock_get_hz(clk_sys) / ((float)ECU_PWM_FREQ_HZ * (float)(ECU_PWM_TOP + 1u));

    pwm_output_init(&g_esc_pwm, ESC_PIN_PWM, ESC_PWM_PERIOD_US - 1u, esc_divider);
    pwm_output_init(&g_ecu_pwm, ECU_PWM_PIN, ECU_PWM_TOP, ecu_divider);
}

static void init_controller(controller_t *controller) {
    memset(controller, 0, sizeof(*controller));
    controller->state = SHIFT_STATE_IDLE;
    controller->gear = restore_gear();
    controller->target_gear = controller->gear;
    controller->esc_current_us = ESC_PWM_NEUTRAL_US;
    controller->esc_target_us = ESC_PWM_NEUTRAL_US;
    controller->esc_ramp_start_us = ESC_PWM_NEUTRAL_US;
    controller->state_started_us = now_us();
    controller->last_status_led_us = controller->state_started_us;

    write_ecu_gear_output(controller->gear);
    esc_set_target(controller, ESC_PWM_NEUTRAL_US, controller->state_started_us);
    update_status_led(controller);

    printf("Initial gear: %s\n", gear_name(controller->gear));
    printf("Persistence: %s, ramping: %s, P control: %s, shift filter: %llu us\n",
           ENABLE_GEAR_PERSISTENCE ? "enabled" : "disabled",
           ENABLE_ESC_RAMP ? "enabled" : "disabled",
           ENABLE_ESC_P_CONTROL ? "enabled" : "disabled",
           (unsigned long long)SHIFT_REQUEST_MIN_ACTIVE_US);
}

int main(void) {
    stdio_init_all();
    sleep_ms(50);

    init_rgb_power();
    init_pwm_outputs();
    init_encoder_pio();
    init_ws2812_pio();
    init_shift_inputs();
    init_controller(&g_controller);

    watchdog_enable(MAIN_LOOP_WATCHDOG_MS, true);

    while (true) {
        const uint64_t loop_started_us = now_us();
        const int32_t encoder_count = quadrature_encoder_get_count(g_encoder_pio, g_encoder_sm);
        const uint64_t timestamp_us = now_us();

        service_shift_requests(timestamp_us);

        const uint64_t state_started_us = timestamp_us;
        service_state_machine(&g_controller, encoder_count, timestamp_us);
        enforce_task_budget(&g_controller, "state machine runtime exceeded", state_started_us, STATE_TASK_BUDGET_US);

        if ((timestamp_us - g_controller.last_status_led_us) >= STATUS_LED_UPDATE_US) {
            const uint64_t led_started_us = now_us();
            update_status_led(&g_controller);
            g_controller.last_status_led_us = timestamp_us;
            enforce_task_budget(&g_controller, "status LED runtime exceeded", led_started_us, LED_TASK_BUDGET_US);
        }

        watchdog_update();
        enforce_task_budget(&g_controller, "main loop runtime exceeded", loop_started_us, SHIFT_TIMEOUT_US);

        sleep_us(LOOP_SLEEP_US);
    }
}
