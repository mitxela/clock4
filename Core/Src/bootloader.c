// This file manages the second half of the chain-loaded bootloader.
// We tell the date side of the display to report its firmware version
// and if needed, tell it to start its system bootloader to do the update.
// The L476 acts as host to the stm32 uart bootloader protocol.


#include "stm32l4xx_hal.h"
#include "bootloader.h"
#include "fatfs.h"

extern UART_HandleTypeDef huart2;
extern CRC_HandleTypeDef hcrc;

#define byteswap32(x) \
   ( ((x & 0xff000000) >> 24) | ((x & 0x00ff0000) >> 8) \
   | ((x & 0x0000ff00) <<  8) | ((x & 0x000000ff) << 24))

// Firmware image on file system fails CRC check
#define ERR_FS_IMG_CRC_INVALID    0b0000011000 // 1

#define DATE_APP_SIZE 32768


void hang_error(uint16_t errno){
  //stopAnimation();

//  buffer_b[0] = bCat0 | 0b0101111000; //d
//  buffer_b[1] = bCat1 | errno;
//
//  buffer_b[4] = bCat4 | 0b0111100100; //E
//  buffer_c[0].low=0b01010000; //r
//  buffer_c[1].low=0b01010000; //r
//  buffer_c[2].low=0b01011100; //o
//  buffer_c[3].low=0b01010000; //r

  while(1){}
}
// Get the CRC of the firmware file on FATFS
// If the CRC is wrong, immediately throw an error.
// The firmware file is padded to a fixed length, we don't need to worry about lengths not divisible by 4.
uint32_t f_crc(FIL* fp)
{
#define READ_BLOCK_SIZE (4096*2)
  unsigned int rc;
  char buf[READ_BLOCK_SIZE];
  uint32_t result;

  if ((fp)->fptr !=0) // pointless but just in case
    f_rewind(fp);

  if ((fp)->obj.objsize != DATE_APP_SIZE)
    hang_error(ERR_FS_IMG_CRC_INVALID);

  hcrc.State = HAL_CRC_STATE_BUSY;
  __HAL_CRC_DR_RESET(&hcrc);

  while ((fp)->fptr < DATE_APP_SIZE -READ_BLOCK_SIZE) {
    f_read(fp, &buf, READ_BLOCK_SIZE, &rc);
    for (int j=0; j<READ_BLOCK_SIZE; j+=4)
      hcrc.Instance->DR = buf[j+3] | (buf[j+2]<<8) | (buf[j+1]<<16) | (buf[j]<<24);
  }
  f_read(fp, &buf, READ_BLOCK_SIZE-4, &rc);
  for (int j=0; j<READ_BLOCK_SIZE-4; j+=4)
    hcrc.Instance->DR = buf[j+3] | (buf[j+2]<<8) | (buf[j+1]<<16) | (buf[j]<<24);

  result = ~(hcrc.Instance->DR);
  hcrc.State = HAL_CRC_STATE_READY;

  f_read(fp, &buf, 4, &rc);
  if (result != (buf[3] | (buf[2]<<8) | (buf[1]<<16) | (buf[0]<<24)) )
    hang_error(ERR_FS_IMG_CRC_INVALID);

  return result;
#undef READ_BLOCK_SIZE
}

void uart2_transmit_byte(uint8_t c){
  while( !( USART2->ISR & USART_ISR_TXE ) ) {};
  USART2->TDR = c;
}

uint8_t wait_for_ack(void){
  uint8_t r;
//  if (HAL_UART_Receive(&huart2, &r, 1, 1000) == HAL_TIMEOUT)
//      return 1;

  while( !( USART2->ISR & USART_ISR_RXNE ) ) {};
  r = USART2->RDR;

  return (r==0x79);
}


uint8_t doDateUpdate(void) {

  FIL file;

//  if (f_open(&file, "/FWD.BIN", FA_READ) != FR_OK)
//    return 1;
//
//  uint32_t new_fw_crc = f_crc(&file);


  uart2_transmit_byte(DATE_CMD_REPORT_CRC);

  uint32_t cur_fw_crc;
  if (HAL_UART_Receive(&huart2, (uint8_t*)&cur_fw_crc, 4, 10) == HAL_TIMEOUT)
    return 2; // date side not responding

  //if (cur_fw_crc == new_fw_crc) return 0;


  uart2_transmit_byte(DATE_CMD_START_BOOTLOADER);

  HAL_Delay(100);

  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.Parity = UART_PARITY_EVEN;
  HAL_UART_Init(&huart2);

  HAL_Delay(100);

  uart2_transmit_byte(0x7F);

  if (wait_for_ack() !=0) return 3; //failed to init bootloader

  HAL_Delay(100);


  uart2_transmit_byte(0x21);
  uart2_transmit_byte(0x21 ^ 0xFF);

  if (wait_for_ack() !=0) return 4;

  uart2_transmit_byte(0x08);
  uart2_transmit_byte(0x00);
  uart2_transmit_byte(0x00);
  uart2_transmit_byte(0x00);
  uart2_transmit_byte(0x08);

//  uart2_transmit_byte(0x00);
//  //HAL_Delay(10);
//  uart2_transmit_byte(0xFF);

  if (wait_for_ack() !=0) return 5;


  return 0;
}
