#ifndef _MEASUREMENTS_H_
#define _MEASUREMENTS_H_


#include <stdint.h>
#include <stdbool.h>


#define ADC_REF_MV   (1000)
#define ADC_MAX_CODE (1024)


typedef union {
    struct {
        uint16_t fun_fb;
        uint16_t heater_tc;
        uint16_t ext_tc;
    } channel_name;
    uint16_t channel_index[3];
} meas_adc_data_t;

extern meas_adc_data_t meas_adc_data;


extern void meas_init(void);
extern bool meas_is_data_ready(void);
extern void meas_process(void);


#endif    // _MEASUREMENTS_H_
