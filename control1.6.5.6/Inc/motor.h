#ifndef __motor_H__
#define __motor_H__

#include "stm32f4xx_hal.h"
#include "main.h"

extern uint16_t vsm1,vsm2,vzm1,vzm2;//pwm���ֵ����ת����1825����ת������1000,vzm1Ϊ��ת���ֵ,vzm2Ϊ��ת���ֵ
extern int16_t pitch,yaw,roll;//Ŀ����̬��
extern int32_t db;
extern int16_t kp[6],ki[6],kd[6];//0Ϊ��ȣ�1Ϊroll��2Ϊpitch��3Ϊ��λ�ǣ�4Ϊǰ���ƣ�5Ϊ�����ƣ�dbΪ����pid����С��ֵ

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
void stp1(void);//ֹͣƽ��
void stp2(void);//ֹͣ��Ǳ


#endif
