#ifndef _FUN_DRIVER_H_
#define _FUN_DRIVER_H_

#include <stdint.h>
#include <stdbool.h>


#define FUN_MAX_SETUP_SPEED (10)


extern uint8_t fun_speed;


extern void fun_driver_init(void);
extern void fun_driver_process(void);


#endif   // _FUN_DRIVER_H_
