#include "fun_driver.h"
#include <stdint.h>
#include <stdbool.h>
#include <avr/io.h>
#include "meas.h"
#include "systimer.h"
#include "error_handler.h"


#define FUN_MIN_VOLTAGE_MV (6 * 1000)
#define FUN_MAX_VOLTAGE_MV (24 * 1000)

#define FUN_FB_DIV (50)

#define FUN_CONTROL_OCR_MIN (0)
#define FUN_CONTROL_OCR_MAX (35)


uint8_t fun_speed;


static timer_t fun_speed_setup_timer;
static const uint32_t fun_fb_min = ((uint32_t)8000 * (uint32_t)ADC_MAX_CODE) / ((uint32_t)ADC_REF_MV * (uint32_t)FUN_FB_DIV);
static const uint32_t fun_fb_max = ((uint32_t)30000 * (uint32_t)ADC_MAX_CODE) / ((uint32_t)ADC_REF_MV * (uint32_t)FUN_FB_DIV);
static bool is_fun_err;



void fun_driver_init(void) {
    // Tim 0 init
    OCR0A = 64;   // TOP
    OCR0B = 0;
    #define WGM0 (5)           // Waveform Generation Mode: 5 - Fast PWM, TOP = OCRA
    TCCR0A = ((WGM0 & 0b11) << WGM00)  |
             (2 << COM0B0) |   // 0 - OCx disconnected, 1-3 - see DS, function of the COM2A1:0 bits depends on the WGM22:0 bit setting. 2 - non-inverting mode, 3 - inverting mode
             (0 << COM0A0);    // 0 - OCx disconnected, 1-3 - see DS, function of the COM2A1:0 bits depends on the WGM22:0 bit setting. 2 - non-inverting mode, 3 - inverting mode
    TCCR0B = (((WGM0 & 0b1100) >> 2) << WGM02)  |
             (1 << CS00);       // Clock Select: 0x00-0x05 -> 0/1/8/32/64/128/256/1024
    // Interrupt Mask Register
    TIMSK0 |= (0 << OCIE0B) |    // Output Compare B Match Interrupt
              (0 << OCIE0A) |    // Output Compare A Match Interrupt
              (0 << TOIE0);      // Overflow Interrupt

    is_fun_err = false;
    fun_speed = 0;
}


void fun_driver_process(void) {
    static uint8_t fun_speed_prev = 0xFF;


    if (fun_speed > FUN_MAX_SETUP_SPEED) fun_speed = FUN_MAX_SETUP_SPEED;
    if (is_fun_err) return;

    if (fun_speed != fun_speed_prev) {
        fun_speed_prev = fun_speed;
        OCR0B = (fun_speed * FUN_CONTROL_OCR_MAX) / FUN_MAX_SETUP_SPEED;
        fun_speed_setup_timer = systimer_set_ms(2000);
    }


    if (!systimer_triggered_ms(fun_speed_setup_timer)) return;
    if ((fun_speed > 4) && (meas_adc_data.channel_name.fun_fb < (uint16_t)fun_fb_min)) {
        eh_state |= EH_STATUS_FLAG_FUN_DCDC_ERR;
        is_fun_err = true;
        OCR0B = 0;
        TCCR0A &= ~(3 << COM0B0); // 0 - OCB disconnected
    }
    if (meas_adc_data.channel_name.fun_fb > (uint16_t)fun_fb_max) {
        eh_state |= EH_STATUS_FLAG_FUN_ERR;
        is_fun_err = true;
        OCR0B = 0;
        TCCR0A &= ~(3 << COM0B0); // 0 - OCB disconnected
    }
}
