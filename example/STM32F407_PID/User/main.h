#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f4xx_hal.h"
#include "./BSP/DC_MOTOR/dc_motor.h"
#include "./BSP/TIMER/dcmotor_tim.h"
#include "general_pid.h"
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
