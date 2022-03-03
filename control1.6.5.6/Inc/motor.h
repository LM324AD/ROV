#ifndef __motor_H__
#define __motor_H__

#include "stm32f4xx_hal.h"
#include "main.h"

extern uint16_t vsm1,vsm2,vzm1,vzm2;//pwm最大值，正转不超1825，反转不低于1000,vzm1为正转最大值,vzm2为反转最大值
extern int16_t pitch,yaw,roll;//目标姿态角
extern int32_t db;
extern int16_t kp[6],ki[6],kd[6];//0为深度，1为roll，2为pitch，3为方位角，4为前后移，5为左右移，db为启动pid的最小阈值

void pid(void);
void up(uint8_t ks);
void down(uint8_t ks);
void roll_left(uint8_t ks);
void roll_right(uint8_t ks);
void foward(uint8_t ks);
void back(uint8_t ks);
void left(uint8_t ks);
void right(uint8_t ks);
void turn_left(void);
void turn_right(void);
void stp1(void);//停止平移
void stp2(void);//停止浮潜


#endif
