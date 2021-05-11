// This file manages the second half of the chain-loaded bootloader.
// We tell the date side of the display to report its firmware version
// and if needed, tell it to start its system bootloader to do the update.
// The L476 acts as host to the stm32 uart bootloader protocol.


#include "stm32l4xx_hal.h"
#include "chainloader.h"
#include "fatfs.h"

extern UART_HandleTypeDef huart2;
extern CRC_HandleTypeDef hcrc;

extern struct {
  uint8_t low;
  uint8_t high;
} buffer_c[5];

extern uint16_t buffer_b[5];

enum {ACK =0, NACK=1};
#define BYTE_ACK   0x79
#define BYTE_NACK  0x1F

#define byteswap32(x) \
   ( ((x & 0xff000000) >> 24) | ((x & 0x00ff0000) >> 8) \
   | ((x & 0x0000ff00) <<  8) | ((x & 0x000000ff) << 24))

// Firmware image on file system fails CRC check
#define ERR_FS_IMG_CRC_INVALID    0b0000011000 // 1
#define ERR_DATE_DEAD             0b0101101100 // 2
#define ERR_GO_FAILED             0b0100111100 // 3

#define DATE_APP_SIZE 32768

void hang_error(uint16_t errno){
  //stopAnimation();

  buffer_b[0] = bCat0 | 0b0101111000; //d
  buffer_b[1] = bCat1 | errno;
  buffer_b[2] = 0;
  buffer_b[3] = 0;

  buffer_b[4] = bCat4 | 0b0111100100; //E

  buffer_c[0].high=0b11011110 ;//| cSegDP;
  buffer_c[0].low=0b01010000; //r
  buffer_c[1].low=0b01010000; //r
  buffer_c[2].low=0b01011100; //o
  buffer_c[3].low=0b01010000; //r

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

void uart2_enable_parity(void){
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.Parity = UART_PARITY_EVEN;
  HAL_UART_Init(&huart2);
}

void boot_cmd(uint8_t c) {
  uart2_transmit_byte(c);
  uart2_transmit_byte(c ^ 0xFF);
}

uint8_t wait_for_ack(uint32_t timeout){
  uint8_t r;
//  if (HAL_UART_Receive(&huart2, &r, 1, 1000) == HAL_TIMEOUT)
//      return 1;

//  while( !( USART2->ISR & USART_ISR_RXNE ) ) {};

  if (UART_WaitOnFlagUntilTimeout(&huart2, UART_FLAG_RXNE, RESET, HAL_GetTick(), timeout) != HAL_OK) {
    return HAL_TIMEOUT;
  }

  r = USART2->RDR;

  return (r!= BYTE_ACK);
}


uint8_t exit_bootloader(void){
  boot_cmd(BOOT_CMD_GO);
  if (wait_for_ack(100) !=0) return NACK;
  uart2_transmit_byte(0x08);
  uart2_transmit_byte(0x00);
  uart2_transmit_byte(0x00);
  uart2_transmit_byte(0x00);
  uart2_transmit_byte(0x08); //checksum
  if (wait_for_ack(100) !=0) return NACK;
  return ACK;
}


uint8_t doDateUpdate(void) {

  FIL file;

//  if (f_open(&file, "/FWD.BIN", FA_READ) != FR_OK)
//    return 1;
//
//  uint32_t new_fw_crc = f_crc(&file);


  // In the situation where we want to force a recovery, the date chip will have jumped to system memory
  // and be awaiting the first byte that allows it to autobaud. We should always send this 0x7F just in case.
  // The main app will ignore it.
  uart2_transmit_byte(0x7F);

  if (wait_for_ack( 10 ) != 0) { // expect timeout, but if we do get an ack skip straight to the update

    uart2_transmit_byte(DATE_CMD_REPORT_CRC);

    uint32_t cur_fw_crc = 0;
    if (HAL_UART_Receive(&huart2, (uint8_t*)&cur_fw_crc, 4, 10) == HAL_TIMEOUT) {
      // Date side not responding - attempt to recover if already in bootloader mode

      if (huart2.RxXferCount == huart2.RxXferSize) {
        // No bytes received
        // If the chip is in bootloader mode it may have interpreted the byte as the start of a 2-byte command
        // Send another byte to force a NACK
        uart2_transmit_byte(DATE_CMD_REPORT_CRC);

        if ((HAL_UART_Receive(&huart2, (uint8_t*)&cur_fw_crc, 1, 10) == HAL_TIMEOUT) || (cur_fw_crc!= BYTE_NACK)) {
          hang_error(ERR_DATE_DEAD);
        }
      } else {
        // Anything other than one byte of nack, give up
        if ((huart2.RxXferCount != huart2.RxXferSize -1) || (cur_fw_crc!= BYTE_NACK))
          hang_error(ERR_DATE_DEAD);
      }
    } else {

      //if (cur_fw_crc == new_fw_crc) return 0;


      uart2_transmit_byte(DATE_CMD_START_BOOTLOADER);

      HAL_Delay(200); // Bootloader takes a fair amount of time to get started

      uart2_transmit_byte(0x7F); //autobaud init byte

      if (wait_for_ack(100) !=0) // failed to init bootloader
        hang_error(ERR_DATE_DEAD); // no other explanation


    }

  }

//  boot_cmd(0x00);
//  if (wait_for_ack() !=0)
//    hang_error(0b0111111100); //8

//  uart2_transmit_byte(0x00);
//  //HAL_Delay(10);
//  uart2_transmit_byte(0xFF);

  //if (wait_for_ack() !=0) return 5;

  HAL_Delay(20);

  if (exit_bootloader() == NACK){
    HAL_Delay(500);
    if (exit_bootloader() == NACK)
      hang_error(ERR_GO_FAILED);
  }

  return 0;
}
