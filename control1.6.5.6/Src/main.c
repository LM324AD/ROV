#include "motor.h"
#include "main.h"

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

CAN_HandleTypeDef hcan1;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

uint8_t u1_l[30]={0xa5,0xdc,0x05,0xdc,0x05,0xdc,0x05,0xdc,0x05,
	                     0x00,0x00,0x6e,0x05,0xc8,0x05,0xb4,0x05,
                       0xdc,0x05,0xdc,0x05,0xdc,0x05,0xdc,0x05,
                       0x00,0x00,0xd0,0x07,0xd8};
uint8_t u1[30],u2[15],u3[14],u5[50],u6[41]={0},u2o[40]={0},u2i[40];
uint8_t u6_l=0,u6_f=0,u2_l=0,u2_f=0,u5_l=0,u5_f=0;
uint8_t led=0,tr=0,fr=0;//tr为计数器,fr为标志位
DEPTH_SENSOR deep_result;
int16_t deepth;
int16_t a[3],w[3],xita[3],t;
uint16_t st[4]={1390,1480,1460,2000},light=1500;//0为舵机1,1为舵机2,2为舵机3,3为机械手舵机

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_IWDG_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_UART5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);

int main(void)
{
	uint16_t i;

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
//  MX_CAN1_Init();
  MX_IWDG_Init();
//  MX_SPI1_Init();
//  MX_TIM1_Init();
  MX_TIM2_Init();
//  MX_TIM4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_ADC1_Init();
//	MX_ADC2_Init();
	for(i=0;i<=5;i++)
	{
//		kp[i]=300;
//		ki[i]=0;
//		kd[i]=1;
//	 kp[i]=((float)flash_read(ADDR_FLASH_SECTOR_3+(i*3+0)+1))/100;
//	 ki[i]=((float)flash_read(ADDR_FLASH_SECTOR_3+(i*3+1)+1))/1000;
//	 kd[i]=((float)flash_read(ADDR_FLASH_SECTOR_3+(i*3+2)+1))/1000;
	}
//	for(i=1;i<=8;i++)
//	{
//		  u1[2*i-1]=1500%256;
//			u1[2*i]=1500>>8;
//	}
//	deepth=0x0155;//测试
  while (1)
  {
		uint32_t AD_Value,AD2_Value,i;
		float t1,d1;
//	uint8_t temp[4];
		
		
		/*采集温度*/
		AD_Value=0;
		HAL_ADC_Start(&hadc1);//adc采集温度数据
    HAL_ADC_PollForConversion(&hadc1,500);          
    if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))
    {
        AD_Value = HAL_ADC_GetValue(&hadc1);
    }  
		t1=AD_Value*3.3/4096;
		t=(t1*1000-500)/10;
		
//		/*采集深度 ms5837*/
//		MS5837_Temperature_Begin_conversion();
//	  MS5837_Temperature_Read();
//  	MS5837_Pressure_Begin_Conversion();
//	  MS5837_Pressure_Read();
//  	deep_result=MS5837_Pressure_ReadAndDMP();
//	  deepth=(depth_caculate(deep_result.P))/10000*100;//cm
		
//		/*采集深度*/
//		AD2_Value=0;
//		HAL_ADC_Start(&hadc2);//adc采集深度数据
//    HAL_ADC_PollForConversion(&hadc2,500);          
//    if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc2), HAL_ADC_STATE_REG_EOC))
//    {
//        AD2_Value = HAL_ADC_GetValue(&hadc2);
//    }  
//		d1=AD2_Value*5/4096;
//		deepth=(d1/5-0.1)*1.5;
		
//		temp[0]=AD_Value;
//		temp[1]=AD_Value>>8;
//		temp[3]=t2;	
//    HAL_UART_Transmit(&huart2,temp+3,1,100);//返回温度数据		
//		for(i=0;i<0xffffff;i++);
  }
  /* USER CODE END 3 */
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	uint8_t i,f;
	if(UartHandle==&huart2)
	{
	 if(u2i[0]==0xfe&&u2_l==0)
		{			
			u2_f^=u2i[u2_l];
			u2_l=1;
			HAL_UART_Receive_IT(&huart2,u2i+u2_l,1);
		}
		else
		 if(u2i[1]==0xfe&&u2_l==1)
		{			
			u2_f^=u2i[u2_l];
			u2_l=2;
			HAL_UART_Receive_IT(&huart2,u2i+u2_l,1);
		}
		else 
		 if(u2i[0]==0xfe&&u2_l>=2)
		{
			
			if(u2_l>=2&&u2_l<21) u2_f+=u2i[u2_l];
			
			u2_l++;
	   if(u2_l==24)
	   {
			 if(u2_f==u2i[21])
			 {
				 
		    IWDG->KR=0xaaaa;//喂狗 
				 fr=1;
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, led);//灯闪烁
		    led=!led;	
        if(u2i[2]==1)
	    	{
					int16_t b1,b2,b3,b4;
				  int16_t a1,a2,a3,a4;
					
		   	  //姿态控制
			    if(u2i[3]>100) foward(abs(u2i[3]-100));
			    else if(u2i[3]<100) back(abs(u2i[3]-100));
					else if(u2i[3]==100&&u2i[4]==100) stp1();
			    if(u2i[5]>100) turn_right();
			    else if(u2i[5]<100) turn_left();
			    if(u2i[4]>100) right(abs(u2i[4]-100));
			    else if(u2i[4]<100) left(abs(u2i[4]-100));
					else if(u2i[4]==100&&u2i[3]==100) stp1();
			    if(u2i[8]>100) up(abs(u2i[8]-100));
			    else if(u2i[8]<100) down(abs(u2i[8]-100));
					else if(u2i[8]==100) stp2();
//					if(u2i[6]>100) roll_right(abs(u2i[6]-100));
//			    else if(u2i[6]<100) roll_left(abs(u2i[6]-100));
					
			    //机械臂控制
				  b1 = u2i[10]+(u2i[9]<<8)-300;
				  b2 = u2i[12]+(u2i[11]<<8)-300;
				  b3 = u2i[14]+(u2i[13]<<8)-300;
				  a1 = b1*360/600;
				  a2 = b2*360/600;
			   	a3 = b3*360/600;
		    
		      st[0]=1390+a1*400/36;
			   	st[1]=1480+a2*400/36;
				  st[2]=1460+a3*400/36;
				  if(u2i[15] == 0x11)  st[3]=2000;//松开,脉宽1ms
				  else if(u2i[15]==0x1f) st[3]=3000;//抓取,脉宽1.5ms
				 
//					st[0]=1500+(((u2i[9]<<8)+u2i[10])-300)*200/36;
//					st[1]=1500+(((u2i[11]<<8)+u2i[12])-300)*200/36;
//					st[2]=1500+(((u2i[13]<<8)+u2i[14])-300)*200/36;
//					if(u2i[15]==0x1f) st[3]=3767;//抓取,脉宽1.5ms
//					else if(u2i[15]==0x11) st[3]=1256;//松开,脉宽0.5ms

			    //大灯控制
					light=1500+u2i[16]*4;
			
		    }
				if(u2i[2]==3)
				{
//					if(flash_write(ADDR_FLASH_SECTOR_3+1,ADDR_FLASH_SECTOR_3+18,u2i+3))
//						u2o[20]=0xff;
//					for(i=0;i<=5;i++)
//	        {
//	          kp[i]=((float)flash_read(ADDR_FLASH_SECTOR_3+(i*3+0)+1))/100;
//	          ki[i]=((float)flash_read(ADDR_FLASH_SECTOR_3+(i*3+1)+1))/1000;
//	          kd[i]=((float)flash_read(ADDR_FLASH_SECTOR_3+(i*3+2)+1))/1000;
//	        }
				}
			 }
			 u2_l=0;
			 u2_f=0;
//			 HAL_UART_Transmit(&huart2,u2i,40,200);//测试
			 u2i[0]=0xff;
			 u2i[1]=0xff;
			 u2i[2]=0xff;
	   }
		 if(u2_l>24)
		 {
			 u2_l=0;
			 u2_f=0;
			 u2i[0]=0xff;
			 u2i[1]=0xff;
		 }
		 HAL_UART_Receive_IT(&huart2,u2i+u2_l,1);
		 return;
    }
		else
		{ 
			 u2_l=0;
  		 u2_f=0; 
			 u2i[0]=0xff;
			 u2i[1]=0xff;
			 u2i[2]=0xff;
		}
//		HAL_UART_Transmit(&huart2,u2i,1,100);//测试
		
	 HAL_UART_Receive_IT(&huart2,u2i,1);
		
  }
	if(UartHandle==&huart3)
	{
		f=0;
	 for(i=1;i<13;i++) f+=u3[i];
	 if(u3[0]==0xa1&&f==u3[13])
	 { 
     //矿石坐标解算，返回机器人当前位置	 
	 }
	 HAL_UART_Receive_IT(&huart3,u3,14);
  }
	if(UartHandle==&huart6)//买完陀螺仪再定数值
	{
//		uint32_t tem;
		if(u6[0]==0x77&&u6_l==0)
		{			
			u6_l=1;
			HAL_UART_Receive_IT(&huart6,u6+1,1);
		}
		else 
		if(u6[0]==0x77&&u6_l>=1)
		{
			if(u6_l<13) u6_f+=u6[u6_l];
			u6_l++;
	   if(u6_l==14)//陀螺仪姿态解算
	   {
		   /*北微陀螺仪*/
			 if(u6_f==u6[13])
			 {
		    xita[1]=((uint8_t)u6[4]%16)*100+((u6[5]>>4))*10+(uint8_t)u6[5]%16+((u6[6]>>4))/10+((uint8_t)u6[6]%16)/100;
		    xita[1]*=(u6[4]>>4)==0?1:(-1);
		    xita[0]=((uint8_t)u6[7]%16)*100+(u6[8]>>4)*10+(uint8_t)u6[8]%16+(u6[9]>>4)/10+((uint8_t)u6[9]%16)/100;
		    xita[0]*=(u6[7]>>4)==0?1:(-1);
		    xita[2]=((uint8_t)u6[10]%16)*100+(u6[11]>>4)*10+(uint8_t)u6[11]%16+(u6[12]>>4)/10+((uint8_t)u6[12]%16)/100;
		    xita[2]*=(u6[10]>>4)==0?1:(-1);
//				HAL_UART_Transmit(&huart2,u6+1,1,100);//测试
//				if(fr==0)
//	      {
//		      pitch=0;
//		      roll=0;
//		      yaw=xita[2];
//					fr=1;
//	      }
			 }
      
			 u6_l=0;
			 u6_f=0;
			 u6[0]=0;
//			HAL_UART_Transmit(&huart2,u6,14,100);//测试
	   }
		 HAL_UART_Receive_IT(&huart6,u6+u6_l,1);
		 return;
    }
	 HAL_UART_Receive_IT(&huart6,u6,1);
	 
  }
	if(UartHandle==&huart5)//深度解算板
	{
//		HAL_UART_Transmit(&huart2,u5,14,100);//测试
//		HAL_UART_Receive_IT(&huart5,u5,14);//测试
		if(u5[0]=='T'&&u5_l==0)
		{			
			u5_l=1;
			HAL_UART_Receive_IT(&huart5,u5+1,1);
		}
		else 
		if(u5[0]=='T'&&u5_l>=1)
		{
//			if(u5_l<15) u5_f+=u5[u5_l];
			u5_l++;
	   if(u5[u5_l-1]==0x0a||u5_l>15)//获取深度
	   {
//			 deepth=0x0155;//测试
			 if(u5[7]=='D'&&u5[8]=='=')
			 {
			  if(u5[11]=='.'&&u5[9]!='-')
			  {
		     deepth=(u5[9]-'0')*1000+(u5[10]-'0')*100+(u5[11]-'0')*10+(u5[12]-'0');
			  }
			  else if(u5[10]=='.')
			  {
				 deepth=(u5[9]-'0')*100+(u5[11]-'0')*10+(u5[12]-'0');
			  }
		   }
      
			 
			 u5_f=0;
			 u5[0]=0;
			 u5[11]=0;
			 u5[10]=0;
			 u5[u5_l]=0;
			 u5_l=0;
//			HAL_UART_Transmit(&huart2,u6,14,100);//测试
	   }
		 HAL_UART_Receive_IT(&huart5,u5+u5_l,1);
		 return;
    }
	 HAL_UART_Receive_IT(&huart5,u5,1);
	 
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	uint8_t i;
	if(htim==&htim2)
	{
		st[0]=1390;
		st[1]=1480;
		st[2]=1460;
		st[3]=2000;
		
		u1[0]=0xa5;
		u1[11]=st[0];
		u1[12]=st[0]>>8;
		u1[13]=st[1];
		u1[14]=st[1]>>8;
		u1[15]=st[2];
		u1[16]=st[2]>>8;
		u1[27]=st[3];
		u1[28]=st[3]>>8;
//		u1[23]=light;
//		u1[24]=light>>8;
		
		pid();//输出数据给电机驱动
		u1[29]=0;
		for(i=1;i<29;i++) u1[29]+=u1[i];
		if(fr)
		 HAL_UART_Transmit(&huart1,u1,30,100);
		else HAL_UART_Transmit(&huart1,u1_l,30,100);
		
		//温度过高过低处理
		//返回数据
		for(i=0;i<32;i++) u2o[i]=0;
		u2o[0]=0xd5;
		u2o[1]=0xd5;
		u2o[2]=((uint16_t)(xita[0]*100))>>8;
		u2o[3]=((uint16_t)(xita[0]*100));
		u2o[4]=((uint16_t)(xita[1]*100))>>8;
		u2o[5]=((uint16_t)(xita[1]*100));
		u2o[6]=((uint16_t)(xita[2]*100))>>8;
		u2o[7]=((uint16_t)(xita[2]*100));
		u2o[8]=((uint16_t)(deepth*23/10))>>8;
		u2o[9]=((uint16_t)(deepth*23/10));
		u2o[10]=((uint16_t)t*100)>>8;
		u2o[11]=((uint16_t)t*100);
		
		u2o[12]=((uint16_t)(kp[1]*10));
		u2o[13]=((uint16_t)(ki[1]*10));
		u2o[14]=((uint16_t)(kd[1]*10));
		u2o[15]=((uint16_t)(kp[2]*10));
		u2o[16]=((uint16_t)(ki[2]*10));
		u2o[17]=((uint16_t)(kd[2]*10));
		u2o[18]=((uint16_t)(kp[3]*10));
		u2o[19]=((uint16_t)(ki[3]*10));
		u2o[20]=((uint16_t)(kd[3]*10));
		u2o[21]=((uint16_t)(kp[0]*10));
		u2o[22]=((uint16_t)(ki[0]*10));
		u2o[23]=((uint16_t)(kd[0]*10));
		u2o[24]=((uint16_t)(kp[4]*10));
		u2o[25]=((uint16_t)(ki[4]*10));
		u2o[26]=((uint16_t)(kd[4]*10));
		u2o[27]=((uint16_t)(kp[5]*10));
		u2o[28]=((uint16_t)(ki[5]*10));
		u2o[29]=((uint16_t)(kd[5]*10));
		
		for(i=2;i<30;i++) u2o[30]+=u2o[i];
		u2o[31]=0xfd;
		u2o[32]=0xfd;
		
		HAL_UART_Transmit(&huart2,u2o,33,100);

	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin==GPIO_PIN_4)//漏水处理
	{
		u2o[19]=1;
	}
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
//    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
//    Error_Handler();
  }
}

static void MX_ADC1_Init(void)
{


  ADC_ChannelConfTypeDef sConfig = {0};

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
//    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
//    Error_Handler();
  }
}

static void MX_ADC2_Init(void)
{


  ADC_ChannelConfTypeDef sConfig = {0};

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
//    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
//    Error_Handler();
  }
}

static void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
//    Error_Handler();
  }

}

static void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_16;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
//    Error_Handler();
  }


}

static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
//    Error_Handler();
  }


}


static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 29999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
//    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
//    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
//    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
//    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
//    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
//    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
//    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
//    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
//    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
}


static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
//    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
//    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
//    Error_Handler();
  }
  HAL_TIM_Base_Start_IT(&htim2);

}

static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
//    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
//    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
//    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
//    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
//    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
//    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
//    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
//    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim4);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);

}

static void MX_UART5_Init(void)
{

  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_MultiProcessor_Init(&huart5, 0, UART_WAKEUPMETHOD_IDLELINE) != HAL_OK)
  {
//    Error_Handler();
  }
  HAL_UART_Receive_IT(&huart5,u5,1);

}

static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_MultiProcessor_Init(&huart1, 0, UART_WAKEUPMETHOD_IDLELINE) != HAL_OK)
  {
//    Error_Handler();
  }

}

static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_MultiProcessor_Init(&huart2, 0, UART_WAKEUPMETHOD_IDLELINE) != HAL_OK)
  {
//    Error_Handler();
  }
  HAL_UART_Receive_IT(&huart2,u2,1);

}

static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_MultiProcessor_Init(&huart3, 0, UART_WAKEUPMETHOD_IDLELINE) != HAL_OK)
  {
//    Error_Handler();
  }
//  HAL_UART_Receive_IT(&huart3,u3,14);

}

static void MX_USART6_UART_Init(void)
{
//	uint8_t tx[6]={0x77,0x05,0x00,0x0c,0x04,0x15};
//	uint8_t tx2[6]={0x77,0x04,0x00,0x0a,0x0e};
//	uint32_t i;

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_MultiProcessor_Init(&huart6, 0, UART_WAKEUPMETHOD_IDLELINE) != HAL_OK)
  {
//    Error_Handler();
  }
//	HAL_UART_Transmit(&huart6,tx,6,100);//设置陀螺仪发送数据方式,发送速率25Hz
//		HAL_UART_Receive_IT(&huart6,u6,6);
//	for(i=1;i<12000000;i++);
//	
//	HAL_UART_Transmit(&huart6,tx2,5,100);//保存设置
//		HAL_UART_Receive_IT(&huart6,u6,6);
	
  HAL_UART_Receive_IT(&huart6,u6,1);

}


static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}



/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
