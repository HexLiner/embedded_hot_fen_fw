#include <stdint.h>
#include <stdbool.h>
#include "device_registers.h"
#include "heater_driver.h"
#include "fun_driver.h"
#include "meas.h"
#include "error_handler.h"


static uint8_t drvice_reg_cmd = 0;


uint8_t *device_registers_ptr[DEVICE_RAM_REG_QTY] = {
    // 0
    (uint8_t*)&drvice_reg_cmd,
    // 1
    (uint8_t*)&heater_setup_temperature_raw + 1,
    (uint8_t*)&heater_setup_temperature_raw + 0,
    // 3
    (uint8_t*)&heater_setup_temperature_c + 1,
    (uint8_t*)&heater_setup_temperature_c + 0,
    // 5
    (uint8_t*)&fun_speed,
    // 6
    (uint8_t*)&heater_ocr_calibr_k + 1,
    (uint8_t*)&heater_ocr_calibr_k + 0,
    // 8
    (uint8_t*)&heater_cal_ocr_minimal_ocr + 1,
    (uint8_t*)&heater_cal_ocr_minimal_ocr + 0,
    // A
    (uint8_t*)&heater_real_temperature_c + 1,
    (uint8_t*)&heater_real_temperature_c + 0,
    // C
    (uint8_t*)&cjs_temperature + 1,
    (uint8_t*)&cjs_temperature + 0,
    // E
    (uint8_t*)&eh_state,
};


void drvice_registers_proc(void) {
    switch (drvice_reg_cmd) {
        case 0:
            break;
        
        case 1:
            is_heater_tc_calibr = true;
            heater_ocr_calibr_k = 1;
            heater_cal_ocr_minimal_ocr = 0;
            heater_setup_temperature_raw = 0;
            break;

        case 2:
            is_heater_tc_calibr = false;
            heater_setup_temperature_c = 0;
            break;

        default:
            break;
    }
    drvice_reg_cmd = 0;
}
