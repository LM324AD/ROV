#include "iic.h"
//#include "delay.h"

void delay_us(uint16_t t)
{
	uint32_t i;
	for(i=0;i<210*t;i++);
}

//��ʼ��IIC
void IIC_Init(void)
{			
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);//IIC_SCL=1
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);//IIC_SDA=1
}
//����IIC��ʼ�ź�
void IIC_Start(void)
{
	uint32_t i;
	SDA_OUT();     //sda�����
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);//IIC_SCL=1
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);//IIC_SDA=1
	delay_us(4);//�ӳ�4us
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);//IIC_SDA=0,START:when CLK is high,DATA change form high to low 
	delay_us(4);//�ӳ�4us
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);//IIC_SCL=0;ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);//IIC_SCL=0;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);//IIC_SDA=0,STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);//IIC_SCL=1; 
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);//IIC_SDA=1;//����I2C���߽����ź�
	delay_us(4);							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
uint8_t IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����  
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);//IIC_SDA=1;
	delay_us(1);	   
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);//IIC_SCL=1;
	delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);//IIC_SCL=0;//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
void IIC_Ack(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);//IIC_SCL=0;
	SDA_OUT();
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);//IIC_SDA=0;
	delay_us(2);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);//IIC_SCL=1;
	delay_us(2);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);//IIC_SCL=0;
}
//������ACKӦ��		    
void IIC_NAck(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);//IIC_SCL=0;
	SDA_OUT();
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);//IIC_SDA=1;
	delay_us(2);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);//IIC_SCL=1;
	delay_us(2);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);//IIC_SCL=0;
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
	SDA_OUT(); 	    
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);//IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, ((txd&0x80)>>7));//IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);   //��TEA5767��������ʱ���Ǳ����
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);//IIC_SCL=1;
		delay_us(2); 
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);//IIC_SCL=0;	
		delay_us(2);
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
uint8_t IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	  {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);//IIC_SCL=0; 
      delay_us(2);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);//IIC_SCL=1;
      receive<<=1;
      if(READ_SDA) receive++;   
		  delay_us(1); 
    }					 
    if (!ack)
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
    return receive;
}

