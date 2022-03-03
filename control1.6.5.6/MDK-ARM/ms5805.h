#ifndef __MS5805_H
#define __MS5805_H
#include "main.h"


/*ms5805��ָ��ĵĶ���*/
#define CMD_RESET  0x1E    //��λ����
#define CMD_PROM  0xA0     //��ȡ�ڲ�flashֵ
#define MS5805_write 0xEC  //д����
#define MS5805_read  0xED  //������
#define High_OSR  0x58     //ת��ʱ��8.22ms(���������ֲ���Եõ�)
#define ACK  1              //����Ӧ��
#define NACK 0              //��Ӧ��
//����ϵ��
typedef struct calibration_words
{
	 uint16_t senst1;           //C1ѹ��������
   uint16_t offt1;            //C2ѹ������ֵ
   uint16_t tcs;              //C3ѹ���������¶�ϵ��
   uint16_t tco;              //C4ѹ�������¶�ϵ��
   uint16_t tref;             //C5�ο��¶�
   uint16_t tempsens;         //C6�¶ȴ������¶�ϵ��
	 uint16_t crc;
} PROM;
//�������ս��
typedef struct{
	uint32_t  P;//ˮѹ
	float  water_temp;//ˮ��
}DEPTH_SENSOR;
/*��������*/
void MS5837_Send_Write_Comend(uint8_t conmend);      //��ms5805��д����
void MS5837_Send_Read_Comend(void);                  //��ms5805�ж�����
void MS5837_PROM_Read(void);                         //��ȡ5805flash���Դ�����ϵ��
void MS5837_Pressure_Read(void);                     //��ȡѹ��ֵ
void MS5837_Temperature_Read(void);                  //��ȡ�¶�ֵ
DEPTH_SENSOR MS5837_Pressure_ReadAndDMP(void);       //�¶�ֵ�Ķ�ȡ�봦��
float depth_caculate(int depth);//һ��ֻ�ܷ�8���ֽ�  //���ֵ�ļ���
void	MS5837_Pressure_Read(void);                    //ѹ���Ķ�ȡ
void	MS5837_Temperature_Begin_conversion(void);     //
void  MS5837_Pressure_Begin_Conversion(void);

#endif


