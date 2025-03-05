#include <stdint.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include "systimer.h"
#include "gpio_driver.h"
#include "cli.h"
#include "encoder_driver.h"
#include "meas.h"
#include "seg_led_driver.h"
#include "fun_driver.h"
#include "heater_driver.h"
#include "eeprom_driver.h"
#include "error_handler.h"
#include "device_registers.h"




#define CLI_ENABLED             (0)
#define HEATER_MIN_SETUP_TEMP_C (50)
const uint32_t sleep_timeout_ms = 1200000;  // 20 minutes


void get_last_ee_settings(uint16_t *setup_heater_temp_c, uint8_t *setup_fun_speed);
void set_last_ee_settings(uint16_t setup_heater_temp_c, uint8_t setup_fun_speed);


int main(void) {
    typedef enum {
        DEVICE_STATE_START,
        DEVICE_STATE_START_VIEW_LAST_SETUP_TEMP,
        DEVICE_STATE_HEAT_TEMP_VIEW,
        DEVICE_STATE_HEAT_TEMP_SET,
        DEVICE_STATE_FUN_SPEED_SET,
        //DEVICE_STATE_SETTINGS_SET,  - unused
        DEVICE_STATE_COOLING_PROC,
        DEVICE_STATE_COOLING_END,
        DEVICE_STATE_CLI_CONTROL,
        DEVICE_STATE_SLEEP_PREPARATION,
        DEVICE_STATE_SLEEP,
        DEVICE_STATE_FATAL,
    } device_state_t;

    device_state_t device_state;
    timer_t device_state_timer;
    timer_t sleep_timer;
    bool is_meas_ready;
    uint16_t setup_heater_temp_c;
    uint8_t setup_fun_speed;
    int8_t encoder_step;
    uint16_t heater_real_temperature_avg_c = 0;
    uint8_t heater_real_temperature_avg_cnt = 0;
    uint8_t board_overtemp_qty = 0;


    // Tim 2 init for systimer
    TCCR2A = 0;
    TCCR2B = (2 << CS20);  // Clock Select: 0x00-0x05 -> 0/1/8/32/64/128/256/1024
    TIMSK2 = (1 << TOIE2);  // TOIE2 irq en 


    gpio_init();
    encoder_init();
    meas_init();
    fun_driver_init();
    heater_init();
    seg_led_init();
    #if (CLI_ENABLED != 0)
    cli_init();
    #endif
    sei();   // global IRQ enable


    get_last_ee_settings(&setup_heater_temp_c, &setup_fun_speed);
    if (eh_state == 0) seg_led_print_intro();
    device_state_timer = systimer_set_ms(4500);
    sleep_timer = systimer_set_ms(sleep_timeout_ms);
    device_state = DEVICE_STATE_START;


    while(1) {
        #if (CLI_ENABLED != 0)
        cli_process();
        drvice_registers_proc();
        #endif
        meas_process();
        encoder_process();
        seg_led_process();

        is_meas_ready = meas_is_data_ready();
        if (is_meas_ready) {
            heater_process();
            fun_driver_process();

            if (cjs_temperature > (80 << 4)) {
                if (board_overtemp_qty >= 40) eh_state |= EH_STATUS_FLAG_BOARD_OVERTEMP_ERR;
                else board_overtemp_qty++;
            }
            else {
                board_overtemp_qty = 0;
            }
        }

        if (eh_state != 0) {
            if (device_state != DEVICE_STATE_FATAL) {
                fun_speed = 0;
                heater_setup_temperature_c = 0;
                seg_led_clear_all_fn();
                seg_led_print_error(eh_state);
                device_state = DEVICE_STATE_FATAL;
            }
        }
        else if (systimer_triggered_ms(sleep_timer)) {
            if ((device_state != DEVICE_STATE_SLEEP) && (device_state != DEVICE_STATE_SLEEP_PREPARATION)) {
                seg_led_clear_all_fn();
                seg_led_print_sleep();
                seg_led_blink_en();
                device_state_timer = systimer_set_ms(40000);
                device_state = DEVICE_STATE_SLEEP_PREPARATION;
            }
        }
        else if (is_heater_tc_calibr) {
            if (device_state != DEVICE_STATE_CLI_CONTROL) {
                seg_led_clear_all_fn();
                seg_led_print_cli();
                device_state = DEVICE_STATE_CLI_CONTROL;
            }
        }


        switch (device_state) {
            case DEVICE_STATE_START:
                if (systimer_triggered_ms(device_state_timer)) {
                    fun_speed = setup_fun_speed;
                    heater_setup_temperature_c = setup_heater_temp_c;
                    encoder_clear_all_events();
                    seg_led_set_fn_heat();
                    seg_led_print_dig(setup_heater_temp_c);
                    device_state_timer = systimer_set_ms(2000);
                    heater_real_temperature_avg_c = 0;
                    heater_real_temperature_avg_cnt = 0;
                    device_state = DEVICE_STATE_START_VIEW_LAST_SETUP_TEMP;
                }
                break;


            case DEVICE_STATE_START_VIEW_LAST_SETUP_TEMP:
                encoder_step = encoder_get_step();

                if (encoder_is_long_press_event()) {
                    seg_led_clear_all_fn();
                    seg_led_print_cool();
                    fun_speed = 10;
                    heater_setup_temperature_c = 0;
                    set_last_ee_settings(setup_heater_temp_c, setup_fun_speed);
                    sleep_timer = systimer_set_ms(sleep_timeout_ms);
                    device_state = DEVICE_STATE_COOLING_PROC;
                }
                else if (encoder_is_press_event()) {
                    seg_led_set_fn_fun();
                    seg_led_print_dig(setup_fun_speed);
                    device_state_timer = systimer_set_ms(4000);
                    sleep_timer = systimer_set_ms(sleep_timeout_ms);
                    device_state = DEVICE_STATE_FUN_SPEED_SET;
                }
                else if (encoder_step != 0) {
                    encoder_step = 0;
                    seg_led_print_dig(setup_heater_temp_c);
                    device_state_timer = systimer_set_ms(4000);
                    sleep_timer = systimer_set_ms(sleep_timeout_ms);
                    device_state = DEVICE_STATE_HEAT_TEMP_SET;
                }
                else if (systimer_triggered_ms(device_state_timer)) {
                    fun_speed = setup_fun_speed;
                    heater_setup_temperature_c = setup_heater_temp_c;
                    seg_led_set_fn_heat();
                    seg_led_print_dig(heater_real_temperature_c);
                    device_state_timer = systimer_set_ms(500);
                    heater_real_temperature_avg_c = 0;
                    heater_real_temperature_avg_cnt = 0;
                    device_state = DEVICE_STATE_HEAT_TEMP_VIEW;
                }
                break;


            case DEVICE_STATE_HEAT_TEMP_VIEW:
                encoder_step = encoder_get_step();

                if (encoder_is_long_press_event()) {
                    seg_led_clear_all_fn();
                    seg_led_print_cool();
                    fun_speed = 10;
                    heater_setup_temperature_c = 0;
                    set_last_ee_settings(setup_heater_temp_c, setup_fun_speed);
                    sleep_timer = systimer_set_ms(sleep_timeout_ms);
                    device_state = DEVICE_STATE_COOLING_PROC;
                }
                else if (encoder_is_press_event()) {
                    seg_led_set_fn_fun();
                    seg_led_print_dig(setup_fun_speed);
                    device_state_timer = systimer_set_ms(4000);
                    sleep_timer = systimer_set_ms(sleep_timeout_ms);
                    device_state = DEVICE_STATE_FUN_SPEED_SET;
                }
                else if (encoder_step != 0) {
                    encoder_step = 0;
                    seg_led_print_dig(setup_heater_temp_c);
                    device_state_timer = systimer_set_ms(4000);
                    sleep_timer = systimer_set_ms(sleep_timeout_ms);
                    device_state = DEVICE_STATE_HEAT_TEMP_SET;
                }
                else if (is_meas_ready) {
                    heater_real_temperature_avg_c += heater_real_temperature_c;
                    heater_real_temperature_avg_cnt++;
                    if (systimer_triggered_ms(device_state_timer)) {
                        device_state_timer = systimer_set_ms(500);
                        heater_real_temperature_avg_c = heater_real_temperature_avg_c / heater_real_temperature_avg_cnt;
                        seg_led_print_dig(heater_real_temperature_avg_c);
                        heater_real_temperature_avg_c = 0;
                        heater_real_temperature_avg_cnt = 0;
                    }
                }
                break;


            case DEVICE_STATE_HEAT_TEMP_SET:
                encoder_step = encoder_get_step();

                if (encoder_is_long_press_event()) {
                    seg_led_clear_all_fn();
                    seg_led_print_cool();
                    fun_speed = 10;
                    heater_setup_temperature_c = 0;
                    set_last_ee_settings(setup_heater_temp_c, setup_fun_speed);
                    device_state = DEVICE_STATE_COOLING_PROC;
                }
                else if (encoder_is_press_event()) {
                    encoder_step = 0;
                    seg_led_set_fn_fun();
                    seg_led_print_dig(setup_fun_speed);
                    device_state_timer = systimer_set_ms(4000);
                    device_state = DEVICE_STATE_FUN_SPEED_SET;
                }
                else if (encoder_step != 0) {
                    if (encoder_step < 0) {
                        /*
                        encoder_step = -encoder_step;
                        if (setup_heater_temp_c < encoder_step) setup_heater_temp_c = 0;
                        else setup_heater_temp_c -= encoder_step;
                        */
                        if (setup_heater_temp_c <= HEATER_MIN_SETUP_TEMP_C) setup_heater_temp_c = HEATER_MIN_SETUP_TEMP_C;
                        else setup_heater_temp_c -= 10;
                    }
                    else {
                        /*
                        setup_heater_temp_c += encoder_step;
                        if (setup_heater_temp_c > HEATER_MAX_SETUP_TEMP_C) setup_heater_temp_c = HEATER_MAX_SETUP_TEMP_C;
                        */
                        if (setup_heater_temp_c >= HEATER_MAX_SETUP_TEMP_C) setup_heater_temp_c = HEATER_MAX_SETUP_TEMP_C;
                        else setup_heater_temp_c += 10;
                    }
                    seg_led_print_dig(setup_heater_temp_c);
                    heater_setup_temperature_c = setup_heater_temp_c;
                    device_state_timer = systimer_set_ms(4000);
                }
                else if (systimer_triggered_ms(device_state_timer)) {
                    device_state_timer = systimer_set_ms(500);
                    heater_real_temperature_avg_c = 0;
                    heater_real_temperature_avg_cnt = 0;
                    device_state = DEVICE_STATE_HEAT_TEMP_VIEW;
                }
                break;


            case DEVICE_STATE_FUN_SPEED_SET:
                encoder_step = encoder_get_step();

                if (encoder_is_long_press_event()) {
                    seg_led_clear_all_fn();
                    seg_led_print_cool();
                    fun_speed = 10;
                    heater_setup_temperature_c = 0;
                    set_last_ee_settings(setup_heater_temp_c, setup_fun_speed);
                    device_state = DEVICE_STATE_COOLING_PROC;
                }
                else if (encoder_is_press_event()) {
                    seg_led_set_fn_heat();
                    seg_led_print_dig(heater_real_temperature_c);
                    device_state_timer = systimer_set_ms(500);
                    heater_real_temperature_avg_c = 0;
                    heater_real_temperature_avg_cnt = 0;
                    device_state = DEVICE_STATE_HEAT_TEMP_VIEW;
                }
                else if (encoder_step != 0) {
                    if (encoder_step < 0) {
                        if (setup_fun_speed > 0) setup_fun_speed--;
                    }
                    else {
                        if (setup_fun_speed < 10) setup_fun_speed++;
                    }
                    seg_led_print_dig(setup_fun_speed);
                    fun_speed = setup_fun_speed;
                    device_state_timer = systimer_set_ms(4000);
                }
                else if (systimer_triggered_ms(device_state_timer)) {
                    seg_led_set_fn_heat();
                    seg_led_print_dig(heater_real_temperature_c);
                    device_state_timer = systimer_set_ms(500);
                    heater_real_temperature_avg_c = 0;
                    heater_real_temperature_avg_cnt = 0;
                    device_state = DEVICE_STATE_HEAT_TEMP_VIEW;
                }
                break;


            case DEVICE_STATE_COOLING_PROC:
                if (encoder_is_long_press_event()) {
                    seg_led_set_fn_heat();
                    seg_led_print_dig(heater_real_temperature_c);
                    fun_speed = setup_fun_speed;
                    heater_setup_temperature_c = setup_heater_temp_c;
                    device_state_timer = systimer_set_ms(500);
                    heater_real_temperature_avg_c = 0;
                    heater_real_temperature_avg_cnt = 0;
                    device_state = DEVICE_STATE_HEAT_TEMP_VIEW;
                }
                else if (is_meas_ready) {
                    if (heater_real_temperature_c <= 40) {
                        seg_led_print_off();
                        fun_speed = 0;
                        device_state = DEVICE_STATE_COOLING_END;
                    }
                }
                break;


            case DEVICE_STATE_COOLING_END:
                if (encoder_is_long_press_event()) {
                    seg_led_set_fn_heat();
                    seg_led_print_dig(heater_real_temperature_c);
                    fun_speed = setup_fun_speed;
                    heater_setup_temperature_c = setup_heater_temp_c;
                    device_state_timer = systimer_set_ms(500);
                    heater_real_temperature_avg_c = 0;
                    heater_real_temperature_avg_cnt = 0;
                    device_state = DEVICE_STATE_HEAT_TEMP_VIEW;
                }
                break;


            case DEVICE_STATE_CLI_CONTROL:
                if (!is_heater_tc_calibr) {
                    heater_setup_temperature_c = setup_heater_temp_c;
                    encoder_clear_all_events();
                    seg_led_set_fn_heat();
                    seg_led_print_dig(heater_real_temperature_c);
                    device_state_timer = systimer_set_ms(500);
                    heater_real_temperature_avg_c = 0;
                    heater_real_temperature_avg_cnt = 0;
                    device_state = DEVICE_STATE_HEAT_TEMP_VIEW;
                }
                break;


            case DEVICE_STATE_SLEEP_PREPARATION:
                encoder_step = encoder_get_step();
                if (encoder_is_press_event() || encoder_is_long_press_event() || (encoder_step != 0)) {
                    seg_led_blink_dis();
                    seg_led_set_fn_heat();
                    seg_led_print_dig(heater_real_temperature_c);
                    fun_speed = setup_fun_speed;
                    heater_setup_temperature_c = setup_heater_temp_c;
                    device_state_timer = systimer_set_ms(500);
                    heater_real_temperature_avg_c = 0;
                    heater_real_temperature_avg_cnt = 0;
                    sleep_timer = systimer_set_ms(sleep_timeout_ms);
                    device_state = DEVICE_STATE_HEAT_TEMP_VIEW;
                }
                else if (systimer_triggered_ms(device_state_timer)) {
                    fun_speed = 1;
                    heater_setup_temperature_c = 0;
                    device_state = DEVICE_STATE_SLEEP;
                }
                break;


            case DEVICE_STATE_SLEEP:
                encoder_step = encoder_get_step();
                if (encoder_is_press_event() || encoder_is_long_press_event() || (encoder_step != 0)) {
                    seg_led_blink_dis();
                    seg_led_set_fn_heat();
                    seg_led_print_dig(heater_real_temperature_c);
                    fun_speed = setup_fun_speed;
                    heater_setup_temperature_c = setup_heater_temp_c;
                    device_state_timer = systimer_set_ms(500);
                    heater_real_temperature_avg_c = 0;
                    heater_real_temperature_avg_cnt = 0;
                    sleep_timer = systimer_set_ms(sleep_timeout_ms);
                    device_state = DEVICE_STATE_HEAT_TEMP_VIEW;
                }
                break;


            case DEVICE_STATE_FATAL:
                break;
                

            default:
                break;
        }
    }

}



ISR(TIMER2_OVF_vect) {
    systimer_process_ms();
}


void get_last_ee_settings(uint16_t *setup_heater_temp_c, uint8_t *setup_fun_speed) {
    uint16_t ee_addr, ee_value;


    for (ee_addr = EE_ADDR_LAST_TEMP_SETUP_BUFF; ee_addr < (EE_ADDR_LAST_TEMP_SETUP_BUFF + EE_LAST_TEMP_SETUP_BUFF_SIZE); ee_addr += 2) {
        eeprom_driver_read_16(ee_addr, &ee_value);
        if (ee_value == 0xFFFF) break;
    }
    if (ee_addr < (EE_ADDR_LAST_TEMP_SETUP_BUFF + EE_LAST_TEMP_SETUP_BUFF_SIZE)) eeprom_driver_read_16((ee_addr - 2), &ee_value);
    *setup_heater_temp_c = ee_value;
    // Check value
    if (*setup_heater_temp_c > HEATER_MAX_SETUP_TEMP_C) *setup_heater_temp_c = HEATER_MIN_SETUP_TEMP_C;
    if (*setup_heater_temp_c < HEATER_MIN_SETUP_TEMP_C) *setup_heater_temp_c = HEATER_MIN_SETUP_TEMP_C;


    for (ee_addr = EE_ADDR_LAST_FUN_SETUP_BUFF; ee_addr < (EE_ADDR_LAST_FUN_SETUP_BUFF + EE_LAST_FUN_SETUP_BUFF_SIZE); ee_addr += 2) {
        eeprom_driver_read_16(ee_addr, &ee_value);
        if (ee_value == 0xFFFF) break;
    }
    if (ee_addr < (EE_ADDR_LAST_FUN_SETUP_BUFF + EE_LAST_FUN_SETUP_BUFF_SIZE)) eeprom_driver_read_16((ee_addr - 2), &ee_value);
    *setup_fun_speed = (uint8_t)ee_value;
    // Check value
    if (*setup_fun_speed > FUN_MAX_SETUP_SPEED) *setup_fun_speed = 4;
}


void set_last_ee_settings(uint16_t setup_heater_temp_c, uint8_t setup_fun_speed) {
    uint16_t ee_addr, ee_value;
    uint16_t prev_setup_heater_temp_c;
    uint8_t prev_setup_fun_speed;


    get_last_ee_settings(&prev_setup_heater_temp_c, &prev_setup_fun_speed);


    if (prev_setup_heater_temp_c != setup_heater_temp_c) {
        for (ee_addr = EE_ADDR_LAST_TEMP_SETUP_BUFF; ee_addr < (EE_ADDR_LAST_TEMP_SETUP_BUFF + EE_LAST_TEMP_SETUP_BUFF_SIZE); ee_addr += 2) {
            eeprom_driver_read_16(ee_addr, &ee_value);
            if (ee_value == 0xFFFF) break;
        }
        if (ee_value != 0xFFFF) ee_addr = EE_ADDR_LAST_TEMP_SETUP_BUFF;

        eeprom_driver_write_16(ee_addr, setup_heater_temp_c);
        ee_addr += 2;
        if (ee_addr < (EE_ADDR_LAST_TEMP_SETUP_BUFF + EE_LAST_TEMP_SETUP_BUFF_SIZE)) {
            eeprom_driver_read_16(ee_addr, &ee_value);
            if (ee_value != 0xFFFF) eeprom_driver_write_16(ee_addr, 0xFFFF);
        }
    }


    if (prev_setup_fun_speed != setup_fun_speed) {
        for (ee_addr = EE_ADDR_LAST_FUN_SETUP_BUFF; ee_addr < (EE_ADDR_LAST_FUN_SETUP_BUFF + EE_LAST_FUN_SETUP_BUFF_SIZE); ee_addr += 2) {
            eeprom_driver_read_16(ee_addr, &ee_value);
            if (ee_value == 0xFFFF) break;
        }
        if (ee_value != 0xFFFF) ee_addr = EE_ADDR_LAST_FUN_SETUP_BUFF;

        eeprom_driver_write_16(ee_addr, setup_fun_speed);
        ee_addr += 2;
        if (ee_addr < (EE_ADDR_LAST_FUN_SETUP_BUFF + EE_LAST_FUN_SETUP_BUFF_SIZE)) {
            eeprom_driver_read_16(ee_addr, &ee_value);
            if (ee_value != 0xFFFF) eeprom_driver_write_16(ee_addr, 0xFFFF);
        }
    }
}
