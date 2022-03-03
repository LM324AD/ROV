
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "motor.h"
#include "ms5805.h"
#include "flash.h"

extern uint8_t u1[30],u2[15],u3[14],u5[50],u6[41],u2o[40];
extern int16_t a[3],w[3],xita[3],deepth;//0ÎªrollÖá,1ÎªpitchÖá,2ÎªzÖá


void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/



#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
