/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdint.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void bootloader_read_uartdata(void);
void bootloader_jump_to_userapplication(void);
/* USER CODE BEGIN EFP */
void bootloader_handle_getver_cmd(uint8_t *bl_rx_buffer);
void bootloader_handle_gethelp_cmd(uint8_t *pBuffer);
void bootloader_handle_getcid_cmd(uint8_t *pBuffer);
void bootloader_handle_getrdp_cmd(uint8_t *pBuffer);
void bootloader_handle_go_cmd(uint8_t *pBuffer);
void bootloader_handle_flash_erase_cmd(uint8_t *pBuffer);
void bootloader_handle_mem_write_cmd(uint8_t *pBuffer);
void bootloader_handle_en_rw_protect(uint8_t *pBuffer);
void bootloader_handle_mem_read (uint8_t *pBuffer);
void bootloader_handle_read_sector_protection_status(uint8_t *pBuffer);
void bootloader_handle_read_otp(uint8_t *pBuffer);
void bootloader_handle_dis_rw_protect(uint8_t *pBuffer);

void bootloader_send_nack();
void bootloader_send_ack(uint8_t len);
void bootloader_write_data(uint8_t *buff,int len);
uint8_t Get_bootloader_version();
uint8_t Verify_host_crc(uint8_t *buffer,uint32_t len,uint32_t hostcrc);
uint32_t bootloader_getdevice_idrev();
uint8_t verify_address(uint32_t addr);
uint8_t bootloader_erase_flash(uint8_t sector_start, uint8_t count);
uint8_t bootloader_write_flash(uint8_t *buff,uint8_t len,uint32_t start_add);
uint8_t bootloader_config_pgm(uint32_t config_data, uint8_t rw_erase_flag);
void bootloader_send_sector_status();
void  bootloader_read_memory(uint32_t address, uint8_t len);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
#define FLASH_SECTOR2_BASE_ADDRESS 0x08008000U
#define BUFFER_LENGTH 200



//#define  <command name >	<command_code>

//This command is used to read the bootloader version from the MCU
#define BL_GET_VER				0x51

//This command is used to know what are the commands supported by the bootloader
#define BL_GET_HELP				0x52

//This command is used to read the MCU chip identification number
#define BL_GET_CID				0x53

//This command is used to read the FLASH Read Protection level.
#define BL_GET_RDP_STATUS		0x54

//This command is used to jump bootloader to specified address.
#define BL_GO_TO_ADDR			0x55

//This command is used to mass erase or sector erase of the user flash .
#define BL_FLASH_ERASE          0x56

//This command is used to write data in to different memories of the MCU
#define BL_MEM_WRITE			0x57

//This command is used to enable or disable read/write protect on different sectors of the user flash .
#define BL_EN_RW_PROTECT		0x58

//This command is used to read data from different memories of the microcontroller.
#define BL_MEM_READ				0x59

//This command is used to read all the sector protection status.
#define BL_READ_SECTOR_P_STATUS	0x5A


//This command is used to read the OTP contents.
#define BL_OTP_READ				0x5B


//This command is used disable all sector read/write protection
#define BL_DIS_R_W_PROTECT		0x5C
/* USER CODE END Private defines */

/* ACK and NACK bytes*/
#define BL_ACK   0XA5
#define BL_NACK  0X7F

/*CRC*/
#define VERIFY_CRC_FAIL    1
#define VERIFY_CRC_SUCCESS 0

/*Version*/
#define BL_VERSION 0x12

//Address valid and invalid

#define ADDR_VALID 1
#define ADDR_INVALID 0

#define INVALID_SECTOR 0x04
#define FLASH_TIMEOUT_VALUE       50000U

#define SRAM1_SIZE            112*1024
#define SRAM1_END             (SRAM1_BASE + SRAM1_SIZE)
#define SRAM2_SIZE            16*1024
#define SRAM2_END             (SRAM2_BASE + SRAM2_SIZE)
#define FLASH_SIZE             1024*1024
#define BKPSRAM_SIZE           4*1024
#define BKPSRAM_END            (BKPSRAM_BASE + BKPSRAM_SIZE)

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
