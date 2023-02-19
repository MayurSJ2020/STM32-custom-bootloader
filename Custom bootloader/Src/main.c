/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
char somedata[] = "Boot loader message\n\r";
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//#define BL_COMMAND_MSG_EN
#define BL_DEBUG_MSG_EN
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

uint8_t bl_rx_buffer[BUFFER_LENGTH];

uint8_t supported_commands[] = {
                               BL_GET_VER ,
                               BL_GET_HELP,
                               BL_GET_CID,
                               BL_GET_RDP_STATUS,
                               BL_GO_TO_ADDR,
                               BL_FLASH_ERASE,
                               BL_MEM_WRITE,
                               BL_READ_SECTOR_P_STATUS} ;

/* USER CODE BEGIN PV */
#define C_UART &huart2
#define D_UART_BL &huart3




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_CRC_Init(void);
static void printmsg(char *format,...);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */

  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) != GPIO_PIN_RESET)
  	{
  		//if user button is pressed during reset we will enter read mode for uart to receive command from the host
	  bootloader_read_uartdata();
  	  printmsg("DEBUG : Button is pressed entered command mode\r\n");

  	}
  	else
  	{
  		//jump to user application
  		printmsg("DEBUG : Button is not pressed jumping to user application \r\n");
  		bootloader_jump_to_userapplication();

  	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1);
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

void MX_USART3_UART_Init(void)
{
	  huart3.Instance = USART3;
	  huart3.Init.BaudRate = 115200;
	  huart3.Init.WordLength = UART_WORDLENGTH_8B;
	  huart3.Init.StopBits = UART_STOPBITS_1;
	  huart3.Init.Parity = UART_PARITY_NONE;
	  huart3.Init.Mode = UART_MODE_TX_RX;
	  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	  if (HAL_UART_Init(&huart3) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  //User push button
  GPIO_InitStruct.Pin  = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed= GPIO_SPEED_FREQ_LOW;

  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/*
 * Print function for debugging via USART3
 */

static void printmsg(char *format,...)
{
	char str[80];
#ifdef BL_DEBUG_MSG_EN
	va_list pooli;
	va_start(pooli,format);
	vsprintf(str,format,pooli);
	HAL_UART_Transmit(D_UART_BL, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
	va_end(pooli);

#endif
}

/*
 * This function is used to read data sent by the host
 * Data format |LENGTH|COMMAND|DATA RELEATED TO COMMAND (IF ANY)|CRC
 * BYTES OF EACH|   1  |    1  |     DEPENDS ON COOMAND			 | 4
 */

void bootloader_read_uartdata(void)
{
	uint8_t receive_length;
	while(1)
	{
	memset(bl_rx_buffer,0,200);
	HAL_UART_Receive(&huart2, bl_rx_buffer, 1, HAL_MAX_DELAY);
	receive_length = bl_rx_buffer[0]; //receiving length of data to be received
	HAL_UART_Receive(&huart2, &bl_rx_buffer[1], receive_length, HAL_MAX_DELAY);//receiving command which is to be executed


	switch(bl_rx_buffer[1])
	{
	 case BL_GET_VER:
		bootloader_handle_getver_cmd(bl_rx_buffer);
		break;
	case BL_GET_HELP:
		bootloader_handle_gethelp_cmd(bl_rx_buffer);
		break;
	case BL_GET_CID:
		bootloader_handle_getcid_cmd(bl_rx_buffer);
		break;
	case BL_GET_RDP_STATUS:
		bootloader_handle_getrdp_cmd(bl_rx_buffer);
		break;
	case BL_GO_TO_ADDR:
		bootloader_handle_go_cmd(bl_rx_buffer);
		break;
	case BL_FLASH_ERASE:
		bootloader_handle_flash_erase_cmd(bl_rx_buffer);
		break;
	case BL_MEM_WRITE:
		bootloader_handle_mem_write_cmd(bl_rx_buffer);
		break;
	case BL_EN_RW_PROTECT:
		bootloader_handle_en_rw_protect(bl_rx_buffer);
		break;
	case BL_MEM_READ:
		bootloader_handle_mem_read(bl_rx_buffer);
		break;
	case BL_READ_SECTOR_P_STATUS:
		bootloader_handle_read_sector_protection_status(bl_rx_buffer);
		break;
	case BL_OTP_READ:
		bootloader_handle_read_otp(bl_rx_buffer);
		break;
	case BL_DIS_R_W_PROTECT:
		bootloader_handle_dis_rw_protect(bl_rx_buffer);
		break;
	 default:
		printmsg("BL_DEBUG_MSG:Invalid command code received from host \n");
		break;
	}
	}
}

/*
 * Application code is present from Flash sector 2
 * Extracting MSP and jumping to reset handler of Application code using function pointer
 */

void bootloader_jump_to_userapplication(void)
{
	//User application reset handler jump function
	void (*Reset_handler_function)(void);

	//Extraction msp value from user application
	uint32_t msp_value = *(volatile uint32_t *) FLASH_SECTOR2_BASE_ADDRESS;

	//Loading extracted MSP value to MSP
	__set_MSP(msp_value);

	uint32_t resethandler_address = *(volatile uint32_t*) (FLASH_SECTOR2_BASE_ADDRESS + 4);

	Reset_handler_function = (void*)resethandler_address;

	//Jumping to user application
	Reset_handler_function();


}

/*
 * Function will get the value of boot loader version and send it to the host
 */

void bootloader_handle_getver_cmd(uint8_t *bl_rx_buffer)
{
	uint8_t bootloader_version;

	uint32_t command_packet_len = bl_rx_buffer[0]+1;
	//Dereferencing the CRC value which was sent by the host |P_LEN|COMMAND|CRC|
	uint32_t hostcrc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	if(!Verify_host_crc(bl_rx_buffer,command_packet_len -4,hostcrc))
	{
		//Proper CRC is received.
		bootloader_send_ack(1);
		bootloader_version = Get_bootloader_version();
		//Sending boot loader version after extracting data.
		bootloader_write_data( &bootloader_version, 1);
	}
	else
	{
		//Sending NACK as host crc does not match with received crc
		bootloader_send_nack();

	}
}
/*
 * This command will help to send all the supported command from the devce
 */
void bootloader_handle_gethelp_cmd(uint8_t *pBuffer)
{
	uint32_t length_received = bl_rx_buffer[0] + 1;

	uint32_t hostcrc = *((uint32_t *)(bl_rx_buffer+length_received-4));

	if(!Verify_host_crc(bl_rx_buffer, length_received-4, hostcrc))
	{
		bootloader_send_ack(sizeof(supported_commands));
		bootloader_write_data(supported_commands, sizeof(supported_commands));
	}
	else
	{
		bootloader_send_nack();
	}
}
/*
 * Function will send chip and revision version to the host
 */
void bootloader_handle_getcid_cmd(uint8_t *pBuffer)
{
	uint32_t device_idrev;
	uint32_t length_received = bl_rx_buffer[0] + 1;

	uint32_t hostcrc = *((uint32_t *)(bl_rx_buffer+length_received-4));

	if(!Verify_host_crc(bl_rx_buffer, length_received-4, hostcrc))
	{
		bootloader_send_ack(4);
		device_idrev = bootloader_getdevice_idrev();
		bootloader_write_data((uint8_t*)&device_idrev, 4);
	}
	else
	{
		bootloader_send_nack();
	}
}
/*
 * This function retrieves the security config option of the device and returns it to the host
 */
void bootloader_handle_getrdp_cmd(uint8_t *pBuffer)
{
		uint8_t rdp_status =0;
		uint32_t length_received = bl_rx_buffer[0] + 1;

		uint32_t hostcrc = *((uint32_t *)(bl_rx_buffer+length_received-4));

		if(!Verify_host_crc(bl_rx_buffer, length_received-4, hostcrc))
		{
			bootloader_send_ack(1);
			//Dereferencing from the config register address
			volatile uint32_t *ptr = (uint32_t*)0x1FFFC000;
			rdp_status = (uint8_t)(* ptr >>8);
			bootloader_write_data(&rdp_status, 1);

		}
		else
		{
			bootloader_send_nack();
		}
}
/*
 * This is used to jump the program into address provided by the host
 * Jumping of application is limited to Flash and RAM
 */
void bootloader_handle_go_cmd(uint8_t *pBuffer)
{

		uint32_t length_received = bl_rx_buffer[0] + 1;
		uint32_t address;
		uint8_t addr_valid = ADDR_VALID;
		uint8_t addr_invalid = ADDR_INVALID;
		uint32_t hostcrc = *((uint32_t *)(bl_rx_buffer+length_received-4));

		if(!Verify_host_crc(bl_rx_buffer, length_received-4, hostcrc))
		{
			bootloader_send_ack(1);
			 address =*((uint32_t *)&bl_rx_buffer[2]);
			 if(verify_address(address) == ADDR_VALID)
			 {
				 bootloader_write_data(&addr_valid, 1);

				 address |= (1<<0); // Setting T bit as this processor does not have arm instruction set

				 void (* Jump_to )(void) = (void *)address;

				 Jump_to();
			 }
			 else
			 {
				 bootloader_write_data(&addr_invalid, 1);
			 }

		}
		else
		{
			bootloader_send_nack();
		}
}
/*
 * This function can handle mass erase and sector erase
 * Host sends starting sector number and count of sectors to be erased
 */
void bootloader_handle_flash_erase_cmd(uint8_t *pBuffer)
{
	uint32_t length_received = bl_rx_buffer[0] + 1;
	uint8_t erase_status = 0x00;
	uint32_t hostcrc = *((uint32_t *)(bl_rx_buffer+length_received-4));

	if(!Verify_host_crc(bl_rx_buffer, length_received-4, hostcrc))
	{
		uint8_t sector_start = bl_rx_buffer[2];
		uint8_t count = bl_rx_buffer[3];

		bootloader_send_ack(1);
		erase_status = bootloader_erase_flash(sector_start,count);
		bootloader_write_data(&erase_status, 1);


	}
	else
	{
		bootloader_send_nack();
	}
}
/*
 * This function is used to program the device using hex or binary file
 * We are using byte programming method data is sent by host
 */
void bootloader_handle_mem_write_cmd(uint8_t *pBuffer)
{
	uint32_t length_received = bl_rx_buffer[0] + 1;
	uint32_t host_crc = *((uint32_t *)(bl_rx_buffer+length_received-4));
	uint8_t payload_len = bl_rx_buffer[6];
	uint32_t payload_add = *((uint32_t *) ( &bl_rx_buffer[2]) );

	uint8_t pgm_status;

	if(!Verify_host_crc(bl_rx_buffer, length_received-4, host_crc))
	{
		bootloader_send_ack(1);
		pgm_status = bootloader_write_flash(&bl_rx_buffer[7],payload_len,payload_add);
		bootloader_write_data(&pgm_status, 1);
	}
	else
	{
		bootloader_send_nack(1);
	}
}
/*
 * This function is used to set the write protection for each sector.
 * Host sends which sector to be protected
 */
void bootloader_handle_en_rw_protect(uint8_t *pBuffer)
{
	uint32_t length_received = bl_rx_buffer[0] + 1;
	uint32_t host_crc = *((uint32_t *)(bl_rx_buffer+length_received-4));
	uint32_t config_data= *((uint16_t *) ( &bl_rx_buffer[2]) );

	uint8_t pgm_status;

	if(!Verify_host_crc(bl_rx_buffer, length_received-4, host_crc))
	{
		bootloader_send_ack(1);
		pgm_status = bootloader_config_pgm(config_data,0);
		bootloader_write_data(&pgm_status, 1);
	}
	else
	{
		bootloader_send_nack(1);
	}
}
/*
 * This function is used to read memory location of the device
 * Access is limited to Flash and RAM location
 */
void bootloader_handle_mem_read (uint8_t *pBuffer)
{
	uint32_t length_received = bl_rx_buffer[0] + 1;
	uint32_t host_crc = *((uint32_t *)(bl_rx_buffer+length_received-4));
	uint32_t address= *((uint32_t *) ( &bl_rx_buffer[2]) );
	uint8_t length = bl_rx_buffer[6];



	if(!Verify_host_crc(bl_rx_buffer, length_received-4, host_crc))
	{
		bootloader_send_ack(length);
	    bootloader_read_memory(address,length);

	}
	else
	{
		bootloader_send_nack(1);
	}
}
/*
 * This function will read the write protection status of the device from the config options and returns it to the device
 */
void bootloader_handle_read_sector_protection_status(uint8_t *pBuffer)
{
	uint32_t length_received = bl_rx_buffer[0] + 1;
	uint32_t host_crc = *((uint32_t *)(bl_rx_buffer+length_received-4));




	if(!Verify_host_crc(bl_rx_buffer, length_received-4, host_crc))
	{
		bootloader_send_ack(2);
		bootloader_send_sector_status();

	}
	else
	{
		bootloader_send_nack(1);
	}
}
/*
 * This function is used to read OTP section of the device
 */
void bootloader_handle_read_otp(uint8_t *pBuffer)
{

}
/*
 * This function will erase all the write protections of all sectors.
 */
void bootloader_handle_dis_rw_protect(uint8_t *pBuffer)
{
	uint32_t length_received = bl_rx_buffer[0] + 1;
	uint32_t host_crc = *((uint32_t *)(bl_rx_buffer+length_received-4));

	uint8_t pgm_status;

	if(!Verify_host_crc(bl_rx_buffer, length_received-4, host_crc))
	{
		bootloader_send_ack(1);
		pgm_status = bootloader_config_pgm(0,1);
		bootloader_write_data(&pgm_status, 1);
	}
	else
	{
		bootloader_send_nack(1);
	}
}
/*
 * =========================================================================================================
 * 													HELPER FUCNTIONS
 * =========================================================================================================
 */

/*
 * Sends NACK to host.
 */

void bootloader_send_nack(void)
{
	uint8_t temp;
	temp = BL_NACK;
	HAL_UART_Transmit(C_UART, &temp, 1, HAL_MAX_DELAY);
}

/*
 * Send ACK to host.
 */

void bootloader_send_ack(uint8_t len)
{
	uint8_t temp[2];
	temp[0] = BL_ACK;
	temp[1] = len;

	bootloader_write_data(temp,2);
}
/*
 * Helper function sends bootloader version to the host
 */
uint8_t Get_bootloader_version()
{
	return BL_VERSION;
}
/*
 * Calculating and verifying host side as well as calculated CRC using CRC engine
 */
uint8_t Verify_host_crc(uint8_t *buffer,uint32_t len,uint32_t hostcrc)
{
	uint32_t device_received_crc = 0xFF;

	 for (uint32_t i=0 ; i < len ; i++)
		{
	        uint32_t i_data = buffer[i];
	        device_received_crc = HAL_CRC_Accumulate(&hcrc, &i_data, 1);
		}

		 /* Reset CRC Calculation Unit */
	  __HAL_CRC_DR_RESET(&hcrc);

		if(device_received_crc == hostcrc)
		{
			return VERIFY_CRC_SUCCESS;
		}

		return VERIFY_CRC_FAIL;
}
/*
 * Sending replies for host issued command using USART2
 */
void bootloader_write_data(uint8_t *buff,int len)
{
	HAL_UART_Transmit(C_UART, buff, len, HAL_MAX_DELAY);
}

/*
 * Command will retrieve device ID and revision code
 */
uint32_t bootloader_getdevice_idrev()
{
	uint32_t device_id = DBGMCU->IDCODE;
	return device_id;
}
/*
 * Helper function which checks the received address which should be in desired range
 */
uint8_t verify_address(uint32_t go_address)
{
	if ( go_address >= SRAM1_BASE && go_address <= SRAM1_END)
		{
			return ADDR_VALID;
		}
		else if ( go_address >= SRAM2_BASE && go_address <= SRAM2_END)
		{
			return ADDR_VALID;
		}
		else if ( go_address >= FLASH_BASE && go_address <= FLASH_END)
		{
			return ADDR_VALID;
		}
		else if ( go_address >= BKPSRAM_BASE && go_address <= BKPSRAM_END)
		{
			return ADDR_VALID;
		}
		else
			return ADDR_INVALID;
}
/*
 * Helper function which erases flash as per received value from the host
 */
uint8_t bootloader_erase_flash(uint8_t sector_start, uint8_t count)
{
	FLASH_EraseInitTypeDef erase_init;
	uint32_t sector_error;
	uint8_t statuss= 9;

	uint8_t remaining_sector;
	remaining_sector = 12 - sector_start;

	if(count >12)
	{
		return INVALID_SECTOR;
	}

	if((sector_start == 0xff) || (sector_start <=11))
	{



	if(sector_start  == (uint8_t)0xFF)
	{
		erase_init.TypeErase = FLASH_TYPEERASE_MASSERASE;
	}
	else
	{
		if(count > remaining_sector)
		{
			count = remaining_sector;
		}
		erase_init.Sector = sector_start;
		erase_init.NbSectors = count;
		erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
	}
	erase_init.VoltageRange = VOLTAGE_RANGE_3;
	erase_init.Banks = FLASH_BANK_1;

	HAL_FLASH_Unlock();
	statuss = (uint8_t)HAL_FLASHEx_Erase(&erase_init,&sector_error);



	HAL_FLASH_Lock();
	return statuss;
	}

	return INVALID_SECTOR;

}
/*
 * Used to write data into flash memory
 */
uint8_t bootloader_write_flash(uint8_t *buff,uint8_t len,uint32_t start_add)
{
	uint8_t status;
	HAL_FLASH_Unlock();
	for(uint32_t i=0;i<len;i++)
	{

		status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, start_add+i, buff[i]);

	}
	HAL_FLASH_Lock();
	return status;
}
/*
 * Used to program write protection bits of the MCU.
 * Granuality number of sectors
 */
uint8_t bootloader_config_pgm(uint32_t config_data, uint8_t rw_erase_flag)
{
	uint32_t * ptr = (uint32_t*) 0x40023C14 ;
	uint8_t flag;
	if(rw_erase_flag)
	{
		flag = HAL_FLASH_OB_Unlock();
		// CHECKING BUSY FLAG IN CONTROL REGISTER
		flag = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE) ;
		// SETTING CONFIG OPTOINS SENT BY THE HOST
		*ptr |= (0xFFF << 16) ;
		//SETTING START OPTION BIT IN OPTCR REGISTER
		*ptr |= (1<<1);
		// CHECKING BUSY FLAG IN CONTROL REGISTER
		flag = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE) ;

		flag = HAL_FLASH_OB_Lock();

		return flag;

	}
	else
	{

		flag = HAL_FLASH_OB_Unlock();

		flag = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE) ;


		//SETTING START OPTION BIT IN OPTCR REGISTER
		config_data = ~(config_data << 16);

		*ptr &= config_data;

		*ptr |= (1<<1);

		flag = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE) ;

		flag = HAL_FLASH_OB_Lock();

		return flag;



	}
}
/*
 * Returns which sectors are write protected and which are not protected
 */
void bootloader_send_sector_status()
{
	uint8_t buff[2] = {0};

	uint32_t *ptr = (uint32_t*)0x1FFFC008;

	buff[0] = (uint8_t)(*ptr );
	buff[1] = (uint8_t)(*ptr >> 8);

	bootloader_write_data(buff, 2);


}

void bootloader_read_memory(uint32_t address, uint8_t len)
{
	uint8_t *ptr = (uint8_t *) address;
	for(int i=0;i<len;i++)
	{
		bootloader_write_data(ptr+i, 1);
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
