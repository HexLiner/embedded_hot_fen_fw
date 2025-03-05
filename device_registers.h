#include <stdint.h>


#ifndef DEVICE_REGISTERS
#define DEVICE_REGISTERS


#define DEVICE_EEPROM_REG_QTY                (1024)
#define DEVICE_RAM_REG_QTY                   (15)

#define EE_ADDR_CALIBR_TC_T1_MEAS_RAW        (0)
#define EE_ADDR_CALIBR_TC_T2_MEAS_RAW        (2)
#define EE_ADDR_OCR_CALIBR_K                 (4)
#define EE_ADDR_OCR_CALIBR_MINIMAL_OCR       (6)
#define EE_ADDR_CALIBR_CHECK_SUM             (8)
#define EE_ADDR_LAST_TEMP_SETUP_BUFF         (0x0100)
#define EE_LAST_TEMP_SETUP_BUFF_SIZE         (0xFF)
#define EE_ADDR_LAST_FUN_SETUP_BUFF          (0x0200)
#define EE_LAST_FUN_SETUP_BUFF_SIZE          (0xFF)


extern uint8_t *device_registers_ptr[DEVICE_RAM_REG_QTY];


extern void drvice_registers_proc(void);


#endif   // DEVICE_REGISTERS
