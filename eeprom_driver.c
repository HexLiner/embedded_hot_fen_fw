#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "eeprom_driver.h"


void eeprom_driver_read_8(uint16_t addr, uint8_t *data) {
    while(EECR & (1 << EEPE)) ;

    char cSREG;
    cSREG = SREG;   // store SREG value
    cli();
    EEARH = (uint8_t)(addr >> 8);
    EEARL = (uint8_t)addr;
    EECR |= (1 << EERE);
    *data = EEDR;
    SREG = cSREG;   // restore SREG value (I-bit)
    sei();
}


void eeprom_driver_read_16(uint16_t addr, uint16_t *data) {
    uint8_t data_h, data_l;


    eeprom_driver_read_8(addr, &data_h);
    eeprom_driver_read_8((addr + 1), &data_l);

    *data = data_h;
    *data = *data << 8;
    *data |= data_l;
}


void eeprom_driver_read(uint16_t addr, uint8_t data_qty, uint8_t *data) {
    uint8_t addr_end;

    addr_end = addr + data_qty;
    for ( ; addr < addr_end; addr++) {
        eeprom_driver_read_8(addr, data);
        data++;
    }
}


void eeprom_driver_write_8(uint16_t addr, uint8_t data) {
    while(EECR & (1 << EEPE)) ;

    char cSREG;
    cSREG = SREG;   // store SREG value
    cli();
    EEARH = (uint8_t)(addr >> 8);
    EEARL = (uint8_t)addr;
    EEDR = data;
    EECR |= (1 << EEMPE);    // must be set after EEPE !!!
    EECR |= (1 << EEPE);
    SREG = cSREG;   // restore SREG value (I-bit)
    sei();
}


void eeprom_driver_write_16(uint16_t addr, uint16_t data) {
    uint8_t data_h, data_l;


    data_h = (uint8_t)(data >> 8);
    data_l = (uint8_t)data;

    eeprom_driver_write_8(addr, data_h);
    eeprom_driver_write_8((addr + 1), data_l);
}


void eeprom_driver_write(uint16_t addr, uint8_t data_qty, const uint8_t *data) {
    uint8_t addr_end;

    addr_end = addr + data_qty;
    for ( ; addr < addr_end; addr++) {
        eeprom_driver_write_8(addr, *data);
        data++;
    }
}
