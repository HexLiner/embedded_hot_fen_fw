#include <stdint.h>


#ifndef ERROR_HANDLING_H
#define ERROR_HANDLING_H


#define EH_STATUS_FLAG_CAL_ERR                (1 << 0)
#define EH_STATUS_FLAG_FUN_DCDC_ERR           (1 << 1)
#define EH_STATUS_FLAG_FUN_ERR                (1 << 2)
#define EH_STATUS_FLAG_HEATER_ERR             (1 << 3)
#define EH_STATUS_FLAG_BOARD_OVERTEMP_ERR     (1 << 4)


extern uint8_t eh_state;


#endif   // ERROR_HANDLING_H
