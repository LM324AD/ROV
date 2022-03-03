#include "iic.h"
#include "ms5805.h"
//#include "delay.h"
#include "math.h"

void delay_ms(uint16_t t)
{
	uint32_t i;
	for(i=0;i<210000*t;i++);
}

/*
*���ܣ���ms5805д����
*������uint8_t conmend    ��Ҫд������
*����ֵ��void
*/
void MS5837_Send_Write_Comend(uint8_t conmend)

{
     IIC_Start();
	   IIC_Send_Byte(0XEC);         //д����
	   IIC_Wait_Ack();              //�ȴ�Ӧ��
     IIC_Send_Byte(conmend);
	   IIC_Wait_Ack();
     IIC_Stop();
}
/*
*����;��ms5805���Ͷ�ָ��
*����;void
*����ֵ;void
*/
void MS5837_Send_Read_Comend(void)
{
     IIC_Start();
	   IIC_Send_Byte(0XED);        //������
	   IIC_Wait_Ack();
}
//��ȡУ׼����
PROM califile;
uint16_t coefficient[8];
uint32_t P_conversion;
uint32_t T_conversion;
/*
*���ܣ���ȡд��flash�еĲ���ϵ��
*������void
*����ֵ��void
*/

void MS5837_PROM_Read(void)
// Initialize library for subsequent pressure measurements
{  
	uint8_t highByte; 
	uint8_t lowByte ;
	uint8_t i;
	MS5837_Send_Write_Comend(CMD_RESET);          //��ȡ֮ǰ���͸�λָ��
	delay_ms(5);
	for(i = 0; i <= 8; i++)
  {
		 
		MS5837_Send_Write_Comend(CMD_PROM + (i*2));  //0xAO-0xAE   
		delay_ms(2);
		MS5837_Send_Read_Comend();
		highByte=IIC_Read_Byte(ACK);
	  lowByte=IIC_Read_Byte(NACK);                //��ȡ���һ���ֽ�ʱ���뷢�ͷ�Ӧ���ź����ͷ�IIC����
		IIC_Stop();
		coefficient[i] = (highByte << 8)|lowByte;
	}
	califile.senst1=coefficient[1];
	califile.offt1=coefficient[2];
	califile.tcs=coefficient[3];
	califile.tco=coefficient[4];
	califile.tref=coefficient[5];
	califile.tempsens=coefficient[6];
	califile.crc=(coefficient[0] >> 12) & 0x000F;
}
/*
*���ܣ���ȡ�¶�ֵ
*������void
*����ֵ��void
*/
void MS5837_Temperature_Read(void)
{
		uint8_t highByte; 
		uint8_t lowByte ;
	  uint8_t middleByte; 
		MS5837_Send_Write_Comend(0x00);//adc��ȡ����
		MS5837_Send_Read_Comend();
		highByte=IIC_Read_Byte(ACK);
	  middleByte=IIC_Read_Byte(ACK);
		lowByte=IIC_Read_Byte(NACK);
		IIC_Stop();
		T_conversion = (highByte << 16)|(middleByte <<8)|(lowByte);
}
/*
*����:d��ȡѹ��ֵ
*������void
*����ֵ��void
*/
void MS5837_Pressure_Read(void)
{
		uint8_t highByte; 
		uint8_t lowByte ;
	  uint8_t middleByte; 
		MS5837_Send_Write_Comend(0x00);
		MS5837_Send_Read_Comend();
		highByte=IIC_Read_Byte(ACK);
	  middleByte=IIC_Read_Byte(ACK);
		lowByte=IIC_Read_Byte(NACK);
		IIC_Stop();
		P_conversion = (highByte << 16)|(middleByte <<8)|(lowByte);
}
/*
*���ܣ����ö�ȡ�¶ȵ�ת���ٶ�
*������void
*����ֵ��void
*/
void MS5837_Temperature_Begin_conversion(void)
{
	MS5837_Send_Write_Comend(0x58);
	delay_ms(20);
}
/*
*���ܣ����ö�ȡѹ����ת���ٶ�
*������void
*����ֵ��void
*/
void MS5837_Pressure_Begin_Conversion(void)
{
	MS5837_Send_Write_Comend(0x48);
  delay_ms(20);
}
// �������һ�ף�ѹ������9800pa   1br= 100 kpa   1mbar=100pa  ÿ����1m����98mbar 1cm ����0.98  ������ΪΪ1mbar
int64_t dT,TEMP;
int64_t off,offi,off2;
int64_t sens,sensi,sens2;
int64_t Ti,T2;
double depth_R;
DEPTH_SENSOR MS5837_Pressure_ReadAndDMP(void)
{
	DEPTH_SENSOR result;
	dT=T_conversion - califile.tref*256;
  sens = (califile.senst1 * pow(2,16))+ ((califile.tcs * dT )/pow(2,7));
  off  = (califile.offt1 * pow(2,17))+ ((califile.tco * dT) / pow(2,6));
	TEMP=2000+(int)(dT*(float)(califile.tempsens/pow(2,23)));
	

//���׻���	
	if(TEMP<2000)//
	{
	 Ti=3*dT*dT/pow(2,33);
	 offi=(3*(TEMP-2000)*(TEMP-2000))/2;
	 sensi=(5*(TEMP-2000)*(TEMP-2000))/8;
	 if(TEMP<-1500)
	 {
		offi=offi+7*(TEMP+1500)*(TEMP+1500);
		sensi=sensi+4*(TEMP+1500)*(TEMP+1500);
	 }
	}
	else
  {
	 Ti=2*dT*dT/pow(2,37);
	 offi=(1*(TEMP-2000)*(TEMP-2000))/16;
	 sensi=0;
	}	
	result.water_temp=TEMP-T2;
	off2=off-offi;
	sens2=sens-sensi;
	result.P = (uint32_t)(((int64_t)P_conversion * sens2 / pow(2,21)- off2)/pow(2,15));//cm
	result.water_temp =TEMP-Ti;
	return result;
}
/*
 * ��������depth_caculate
 * ����  ����ȼ���
 * ����  ����
 * ���  ����
 */  
uint8_t init_flag;
 int depth_base=0;
int depth_num;
int deep_last=0;

float depth_caculate(int depth)//һ��ֻ�ܷ�8���ֽ�
{     
 float deep=0;
	    if(!init_flag)
	   {
		     depth_base+=depth;
			   depth_num++;
			  if(depth_num>=10)
				{
				  init_flag=1;
					depth_base/=10;
				}
						 return 0;
		 }
		 else
		 {	 
				deep=deep*0.2+deep_last*0.8;//��ʼ����deep_lastһ����0
			  deep_last=deep;
				deep=(float)(((float)depth-depth_base)/0.98);//
		 }
//		 HAL_GPIO_TogglePin(GPIOE,SENSOR_TEST_LED1_Pin);
		 return deep;
}
