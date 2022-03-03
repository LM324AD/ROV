#include "flash.h"

uint32_t FirstSector = 0, NbOfSectors = 0, Address = 0;
uint32_t SectorError = 0;
__IO uint8_t data32 = 0 , MemoryProgramStatus = 0;

static uint32_t GetSector(uint32_t Address)//��ȡ��ַ��������
{
  uint32_t sector = 0;
  
  if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
  {
    sector = FLASH_SECTOR_0;  
  }
  else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
  {
    sector = FLASH_SECTOR_1;  
  }
  else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
  {
    sector = FLASH_SECTOR_2;  
  }
  else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
  {
    sector = FLASH_SECTOR_3;  
  }
  else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
  {
    sector = FLASH_SECTOR_4;  
  }
  else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
  {
    sector = FLASH_SECTOR_5;  
  }
  else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
  {
    sector = FLASH_SECTOR_6;  
  }
  else if((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
  {
    sector = FLASH_SECTOR_7;  
  }
  else if((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8))
  {
    sector = FLASH_SECTOR_8;  
  }
  else if((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9))
  {
    sector = FLASH_SECTOR_9;  
  }
  else if((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10))
  {
    sector = FLASH_SECTOR_10;  
  }
  else /* (Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_11) */
  {
    sector = FLASH_SECTOR_11;
  }

  return sector;
}

static uint32_t GetSectorSize(uint32_t Sector)//��ȡ������С
{
  uint32_t sectorsize = 0x00;

  if((Sector == FLASH_SECTOR_0) || (Sector == FLASH_SECTOR_1) || (Sector == FLASH_SECTOR_2) || (Sector == FLASH_SECTOR_3))
  {
    sectorsize = 16 * 1024;
  }
  else if(Sector == FLASH_SECTOR_4)
  {
    sectorsize = 64 * 1024;
  }
  else
  {
    sectorsize = 128 * 1024;
  }  
  return sectorsize;
}

uint16_t MEM_If_Init_FS(void)//����������쳣��־λ 
{
 
    HAL_FLASH_Unlock(); 
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
                          FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
	return 0;
}

uint16_t MEM_If_Erase_FS(uint32_t start_Add,uint32_t end_Add)//��������
{
    /* USER CODE BEGIN 3 */
    uint32_t UserStartSector;
    uint32_t SectorError;
    FLASH_EraseInitTypeDef pEraseInit;
 
    /* Unlock the Flash to enable the flash control register access *************/
    MEM_If_Init_FS();//����
 
    /* Get the sector where start the user flash area */
    UserStartSector = GetSector(start_Add);//��ȡ��ʼ��ַ��������
 
    pEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;//����������
    pEraseInit.Sector = UserStartSector;//��ʼ��ַ��������
    pEraseInit.NbSectors = GetSector(end_Add)-UserStartSector+1 ;//����������
    pEraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;//������ѹ��2.7~3.6V
 
    if (HAL_FLASHEx_Erase(&pEraseInit, &SectorError) != HAL_OK)
    {
        /* Error occurred while page erase */
        return 1;
    }
		
		__HAL_FLASH_DATA_CACHE_DISABLE();//�����������ݻ���
    __HAL_FLASH_INSTRUCTION_CACHE_DISABLE();//��������ָ���

    __HAL_FLASH_DATA_CACHE_RESET();//�����������ݻ��棨ֻ���ڽ����������ݻ����ʹ�ã�
    __HAL_FLASH_INSTRUCTION_CACHE_RESET();//��������ָ��棨ֻ���ڽ�������ָ����ʹ�ã�

    __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();//ʹ���������ݻ���
    __HAL_FLASH_DATA_CACHE_ENABLE();//ʹ���������ݻ���
 
    return 0;
    /* USER CODE END 3 */
}

uint16_t MEM_If_Write_FS(uint32_t start_Add,uint32_t end_Add,uint8_t *DATA_8)//д�����
{
	int16_t i=0;
	
    Address = start_Add;

  while (Address < end_Add)
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, Address, *(DATA_8+i)) == HAL_OK)
    {
      Address = Address + 1;
			i++;
    }
    else
    { 
			return 1;
      /* Error occurred while writing data in Flash memory. 
         User can add here some code to deal with this error */
      /*
        FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError();
      */
//      Error_Handler();
    }
  }

  HAL_FLASH_Lock(); //��������

	/*�����������Ƿ�׼ȷ��
	       MemoryProgramStatus = 0:��������׼ȷ
	       MemoryProgramStatus != 0:�������ݲ�׼ȷ*/
  Address = start_Add;
  MemoryProgramStatus = 0x0;
	i=0;
  
  while (Address < end_Add)
  {
    data32 = *(__IO uint8_t*)Address;

    if (data32 != *(DATA_8+i))
    {
      MemoryProgramStatus++;  
    }
    i++;
    Address = Address + 4;
  }  

  /* Check if there is an issue to program data */
  if (MemoryProgramStatus == 0)
  {
		
    /* No error detected. Switch on LED4 */
    //BSP_LED_On(LED4);
  }
  else
  {
		return 1;
    /* Error detected. Switch on LED5 */
//    Error_Handler();
  }
	return 0;
}

uint8_t flash_write(uint32_t start_Add,uint32_t end_Add,uint8_t *DATA_32)
{
	if(MEM_If_Erase_FS(start_Add,end_Add)||MEM_If_Write_FS( start_Add, end_Add, DATA_32))
	  return 1;
	return 0;
}

uint8_t flash_read(uint32_t Add)//ÿ�ζ�4�ֽڣ�����һ������ַҪ��4
{
	data32 = *(__IO uint8_t*)Add;
	return data32;
}
