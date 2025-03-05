#include "meas.h"
#include <stdint.h>
#include <stdbool.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "systimer.h"


// Reference Selection: 0 - AREF, 1 - AVCC, 2 - reserved, 3 - Internal 1.1V Voltage Reference
//
#define ADMUX_INIT_VAL  ((3 << REFS0) | \
                         (0 << ADLAR))
// ADC Enable
// ADC Start Conversion
// ADC Auto Trigger Enable
// ADC Interrupt Flag
// ADC Interrupt Enable
// ADC Prescaler Select Bits: 2/2/4/8/16/32/64/128
#define ADCSRA_INIT_VAL ((1 << ADEN)  | \
                         (1 << ADSC)  | \
                         (0 << ADATE) | \
                         (0 << ADIF)  | \
                         (1 << ADIE)  | \
                         (4 << ADPS0))


#define MEAS_CHANNELS_QTY (3)
static const uint8_t meas_adc_channels[MEAS_CHANNELS_QTY] = {1, 2, 3};   // FUN_FB, HEATER_TC, EXT_TC
static uint8_t meas_channels_cnt;
static bool is_data_ready;
///tatic uint8_t meas_skip_cnt;
static timer_t meas_process_timer;


meas_adc_data_t meas_adc_data;




void meas_init(void) {
    ADMUX = ADMUX_INIT_VAL;
    DIDR0 = 0;   // Digital Input Disable Register 0

    // Start ADC in Single Conversion mode
    ADCSRA = ADCSRA_INIT_VAL;
    meas_channels_cnt = 0;
    is_data_ready = false;
    ///meas_skip_cnt = 0;

    meas_process_timer = systimer_set_ms(50);
}


bool meas_is_data_ready(void) {
    bool result = is_data_ready;
    is_data_ready = false;
    return result;
}


void meas_process(void) {
    if (systimer_triggered_ms(meas_process_timer)) {
        if (meas_channels_cnt != 0) return;
        meas_process_timer = systimer_set_ms(100);
        TCCR0A &= ~(3 << COM0B0); // 0 - OCB disconnected
        // Start ADC in Single Conversion mode
        ADCSRA = ADCSRA_INIT_VAL;
    }
}



ISR(ADC_vect) {
    uint8_t adc_h, adc_l;


    adc_l = ADCL;
    adc_h = ADCH & 0b11;

    meas_adc_data.channel_index[meas_channels_cnt] = ((uint16_t)adc_h << 8) | adc_l;
    /*
    meas_skip_cnt++;
    if (meas_skip_cnt < 8) {
        // Start ADC in Single Conversion mode
        ADCSRA = ADCSRA_INIT_VAL;
        return;
    }
    meas_skip_cnt = 0;*/

    // Changed ADC channel
    meas_channels_cnt++;
    if (meas_channels_cnt >= MEAS_CHANNELS_QTY) {
        TCCR0A |= (2 << COM0B0); // 0 - OCB en
        // All ADC channels are ready
        meas_channels_cnt = 0;
        is_data_ready = true;
        ADMUX = ADMUX_INIT_VAL | (meas_adc_channels[meas_channels_cnt] << MUX0);
    }
    else {
        ADMUX = ADMUX_INIT_VAL | (meas_adc_channels[meas_channels_cnt] << MUX0);
        // Start ADC in Single Conversion mode
        ADCSRA = ADCSRA_INIT_VAL;
    }
}
