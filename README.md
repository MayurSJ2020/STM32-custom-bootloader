# STM32-custom-bootloader
Developed a custom bootloader which uses uart to communicate with the device. Host talks to bootloader and sends command using console 

Listed of supported commands for bootloader 
   BL_GET_VER                            --> 1 Gets bootloader version
   BL_GET_HLP                            --> 2 Sends all the supported commands by bootloader
   BL_GET_CID                            --> 3 Sends Chip ID
   BL_GET_RDP_STATUS                     --> 4 Gets RDP status of the device
   BL_GO_TO_ADDR                         --> 5 Jump to specified address
   BL_FLASH_MASS_ERASE                   --> 6 Mass erase the device
   BL_FLASH_ERASE                        --> 7 Sector erase the defined sectors
   BL_MEM_WRITE                          --> 8 Memory write using the binary file
   BL_EN_R_W_PROTECT                     --> 9 To enable write protection for selected sectors
   BL_MEM_READ                           --> 10 To read the desired memory of flash
   BL_READ_SECTOR_P_STATUS               --> 11 Read back write protection status of the device WRPi bits
   BL_OTP_READ                           --> 12 Otp section read
   BL_DIS_R_W_PROTECT                    --> 13 Disable all write protections
   BL_MY_NEW_COMMAND                     --> 14 For future implemetation
   MENU_EXIT                             --> 0
   
   Host sends command via USB and it receives using UART 
   Host processes and sends command using python script
