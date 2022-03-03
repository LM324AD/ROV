#ifndef __MS5805_H
#define __MS5805_H
#include "main.h"


/*ms5805关指令的的定义*/
#define CMD_RESET  0x1E    //复位命令
#define CMD_PROM  0xA0     //读取内部flash值
#define MS5805_write 0xEC  //写命令
#define MS5805_read  0xED  //读命令
#define High_OSR  0x58     //转换时间8.22ms(根据数据手册可以得到)
#define ACK  1              //产生应答
#define NACK 0              //无应答
//补偿系数
typedef struct calibration_words
{
	 uint16_t senst1;           //C1压力灵敏度
   uint16_t offt1;            //C2压力补偿值
   uint16_t tcs;              //C3压力灵敏度温度系数
   uint16_t tco;              //C4压力补偿温度系数
   uint16_t tref;             //C5参考温度
   uint16_t tempsens;         //C6温度传感器温度系数
	 uint16_t crc;
} PROM;
//储存最终结果
typedef struct{
	uint32_t  P;//水压
	float  water_temp;//水温
}DEPTH_SENSOR;
/*函数声明*/
void MS5837_Send_Write_Comend(uint8_t conmend);      //向ms5805中写数据
void MS5837_Send_Read_Comend(void);                  //向ms5805中读数据
void MS5837_PROM_Read(void);                         //读取5805flash中自带修正系数
void MS5837_Pressure_Read(void);                     //读取压力值
void MS5837_Temperature_Read(void);                  //读取温度值
DEPTH_SENSOR MS5837_Pressure_ReadAndDMP(void);       //温度值的读取与处理
float depth_caculate(int depth);//一次只能发8个字节  //深度值的计算
void	MS5837_Pressure_Read(void);                    //压力的读取
void	MS5837_Temperature_Begin_conversion(void);     //
void  MS5837_Pressure_Begin_Conversion(void);

#endif


