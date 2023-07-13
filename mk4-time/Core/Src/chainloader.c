// This file manages the second half of the chain-loaded bootloader.
// We tell the date side of the display to report its firmware version
// and if needed, tell it to start its system bootloader to do the update.
// The L476 acts as host to the stm32 uart bootloader protocol.


#include "main.h"
#include "chainloader.h"
#include "fatfs.h"

extern UART_HandleTypeDef huart2;
extern CRC_HandleTypeDef hcrc;

enum {ACK =0, NACK=1};
#define BYTE_ACK   0x79
#define BYTE_NACK  0x1F

#define ERR_FS_IMG_CRC_INVALID    bSegDecode1
#define ERR_DATE_DEAD             bSegDecode2
#define ERR_GET                   bSegDecode3
#define ERR_UNEXPECTED_TIMEOUT    bSegDecode4
#define ERR_VERSION               bSegDecode5
#define ERR_ERASE_FAILED          bSegDecode6
#define ERR_WRITE_PAGE            bSegDecode7
#define ERR_WRITE                 bSegDecode8
#define ERR_GO_FAILED             bSegDecode9

#define DATE_APP_SIZE 32768
#define TIME_APP_SIZE 192*1024

uint32_t cur_fwd_crc = 0;

void hang_error(uint16_t errno){
  //stopAnimation();
  setDisplayPWM(5);

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

  while ((fp)->fptr < DATE_APP_SIZE -READ_BLOCK_SIZE && !fp->err) {
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
  if (fp->err || result != (buf[3] | (buf[2]<<8) | (buf[1]<<16) | (buf[0]<<24)) )
    hang_error(ERR_FS_IMG_CRC_INVALID);

  return result;
#undef READ_BLOCK_SIZE
}

void uart2_transmit_byte(uint8_t c){
  while( !( USART2->ISR & USART_ISR_TXE ) ) {};
  USART2->TDR = c;
}

uint8_t wait_for_ack(uint32_t timeout){
  uint8_t r;

  if (UART_WaitOnFlagUntilTimeout(&huart2, UART_FLAG_RXNE, RESET, HAL_GetTick(), timeout) != HAL_OK) {
    return HAL_TIMEOUT;
  }

  r = USART2->RDR;

  return (r!= BYTE_ACK);
}

uint8_t boot_cmd(uint8_t c) {
  uart2_transmit_byte(c);
  uart2_transmit_byte(c ^ 0xFF);
  return wait_for_ack(10);
}

uint8_t send_addr(uint32_t addr){
  uint8_t b1 = (addr>>24) & 0xFF;
  uint8_t b2 = (addr>>16) & 0xFF;
  uint8_t b3 = (addr>> 8) & 0xFF;
  uint8_t b4 =  addr      & 0xFF;

  uart2_transmit_byte(b1);
  uart2_transmit_byte(b2);
  uart2_transmit_byte(b3);
  uart2_transmit_byte(b4);
  uart2_transmit_byte(b1^b2^b3^b4);
  return wait_for_ack(100);
}

uint8_t read_byte(void){
  if (UART_WaitOnFlagUntilTimeout(&huart2, UART_FLAG_RXNE, RESET, HAL_GetTick(), 10) != HAL_OK)
    hang_error(ERR_UNEXPECTED_TIMEOUT);

  return USART2->RDR;
}

uint8_t write_page(uint32_t addr, uint8_t* data){
  if (boot_cmd(BOOT_CMD_WRITE)!=0)
    hang_error(ERR_WRITE_PAGE);

  if (send_addr(addr)!=0)
    hang_error(ERR_WRITE_PAGE);

  uint8_t chk = 0xFF;
  uart2_transmit_byte(0xFF); // N

  for (uint16_t i = 0; i<256; i++) {
    chk ^= data[i];
    uart2_transmit_byte(data[i]);
  }
  uart2_transmit_byte(chk);

  return wait_for_ack(20000);
}

uint8_t exit_bootloader(void){
  if (boot_cmd(BOOT_CMD_GO) !=0) return NACK;
  return send_addr(0x08000000);
}


#define CHUNK (DATE_APP_SIZE / (8*9))
void progressBar( uint32_t addr, uint32_t * threshold, char * progress ){
  if (addr>*threshold) {
    switch ((++*progress) & 0xF8){
      case 0x00>>1:  buffer_b[0 + 5*(*progress & 0x07)] = bCat0 | 0b0100000000; break;
      case 0x10>>1:  buffer_b[1 + 5*(*progress & 0x07)] = bCat1 | 0b0100000000; break;
      case 0x20>>1:  buffer_b[2 + 5*(*progress & 0x07)] = bCat2 | 0b0100000000; break;
      case 0x30>>1:  buffer_b[3 + 5*(*progress & 0x07)] = bCat3 | 0b0100000000; break;
      case 0x40>>1:  buffer_b[4 + 5*(*progress & 0x07)] = bCat4 | 0b0100000000; break;
      case 0x50>>1:  buffer_c[0 + 5*(*progress & 0x07)].low =     0b01000000  ; break;
      case 0x60>>1:  buffer_c[1 + 5*(*progress & 0x07)].low =     0b01000000  ; break;
      case 0x70>>1:  buffer_c[2 + 5*(*progress & 0x07)].low =     0b01000000  ; break;
      case 0x80>>1:  buffer_c[3 + 5*(*progress & 0x07)].low =     0b01000000  ; break;
    }
    *threshold += CHUNK;
  }
}


uint8_t doDateUpdate(void) {

  // Possible situations for date chip:
  // - normal app loaded, should respond to our request for CRC with four bytes
  //     (will ignore the 0x7F we send)
  // - system bootloader started, awaiting autobaud byte (i.e. manually triggered with boot0 pin)
  // - system bootloader loaded, ready for command
  //     Sending the 0x7F and a dummy byte will cause nack, then ready
  // - system bootloader loaded, halfway through command
  //     Sending 0x7F will immediately nack
  // - system bootloader misconfigured (by launching it while time side was still transmitting data)
  //     Once the time side hangs on error, the date side can be manually restarted with boot0 before reseting the time side.

  FIL file;

  if (f_open(&file, "/FWD.BIN", FA_READ) != FR_OK) {
    // if no new firmware file on fatfs, then we are powerless to do anything
    return 1;
  }

  uint32_t new_fwd_crc = f_crc(&file);


  // In the situation where we want to force a recovery, the date chip will have jumped to system memory
  // and be awaiting the first byte that allows it to autobaud. We should always send this 0x7F just in case.
  // The main app will ignore it.
  uart2_transmit_byte(0x7F);

  if (wait_for_ack( 10 ) != 0) { // expect timeout, but if we do get an ack skip straight to the update

    uart2_transmit_byte(DATE_CMD_REPORT_CRC);

    if (HAL_UART_Receive(&huart2, (uint8_t*)&cur_fwd_crc, 4, 10) == HAL_TIMEOUT) {
      // Date side not responding - attempt to recover if already in bootloader mode

      if (huart2.RxXferCount == huart2.RxXferSize) {
        // No bytes received
        // If the chip is in bootloader mode it may have interpreted the byte as the start of a 2-byte command
        // Send another byte to force a NACK
        uart2_transmit_byte(DATE_CMD_REPORT_CRC);

        if ((HAL_UART_Receive(&huart2, (uint8_t*)&cur_fwd_crc, 1, 10) == HAL_TIMEOUT) || (cur_fwd_crc!= BYTE_NACK)) {
          hang_error(ERR_DATE_DEAD);
        }
      } else {
        // Anything other than one byte of nack, give up
        if ((huart2.RxXferCount != huart2.RxXferSize -1) || (cur_fwd_crc!= BYTE_NACK))
          hang_error(ERR_DATE_DEAD);
      }
    } else {

      if (byteswap32(cur_fwd_crc) == new_fwd_crc) return 0;


      uart2_transmit_byte(DATE_CMD_START_BOOTLOADER);

      HAL_Delay(200); // Bootloader takes a fair amount of time to get started

      uart2_transmit_byte(0x7F); //autobaud init byte

      if (wait_for_ack(100) !=0) // failed to init bootloader
        hang_error(ERR_DATE_DEAD); // no other explanation

    }
  }



  if (boot_cmd(BOOT_CMD_GET)!=0)
    hang_error(ERR_GET);

  uint8_t data[14];

  if (HAL_UART_Receive(&huart2, data, 14, 10) == HAL_TIMEOUT)
    hang_error(ERR_UNEXPECTED_TIMEOUT);

  if (data[8] != BOOT_CMD_EXTENDED_ERASE)
    hang_error(ERR_VERSION);







  // Mass erase is possibly unsupported by L010
  // L010 Flash pages are 128 bytes, => 32kb is 256 pages

  if (boot_cmd(BOOT_CMD_EXTENDED_ERASE)!=0)
    hang_error(ERR_ERASE_FAILED);

#define PAGES_TO_ERASE (DATE_APP_SIZE / 128)

  uart2_transmit_byte((PAGES_TO_ERASE-1) >> 8);
  uart2_transmit_byte((PAGES_TO_ERASE-1) & 0xFF);
  uint8_t chk=((PAGES_TO_ERASE-1)>>8) ^ ((PAGES_TO_ERASE-1) & 0xFF);

  for (uint16_t i=0;i< PAGES_TO_ERASE;i++){
    uart2_transmit_byte((i>>8) & 0xFF);
    chk ^= (i>>8) & 0xFF;
    uart2_transmit_byte(i & 0xFF);
    chk ^= i & 0xFF;
  }
  uart2_transmit_byte(chk);

  // erase can take a long time
  if (wait_for_ack(20000) !=0)
    hang_error(ERR_ERASE_FAILED);



  // Set up display for fading individual segments
  buffer_c[0].low=0;
  buffer_c[1].low=0;
  buffer_c[2].low=0;
  buffer_c[3].low=0;
  buffer_c[4].low=0;
  buffer_b[0] = 0;
  buffer_b[1] = 0;
  buffer_b[2] = 0;
  buffer_b[3] = 0;
  buffer_b[4] = 0;
  for (uint8_t i=0;i<40;i+=5) {
    buffer_c[0 + i].high=0b11001110;
    buffer_c[1 + i].high=0b11001101;
    buffer_c[2 + i].high=0b11001011;
    buffer_c[3 + i].high=0b11000111;
    buffer_c[4 + i].high=0b11001111;
  }
  setDisplayPWM(40);





  uint32_t threshold = 0;
  char progress = 0;

  if (file.fptr !=0)
    f_rewind(&file);

  for (uint32_t addr=0; addr < DATE_APP_SIZE; addr+=256) {
    uint8_t data[256];
    unsigned int rc;
    f_read(&file, data, 256, &rc);
    if (write_page(addr +0x08000000, data) !=0)
      hang_error(ERR_WRITE);
    progressBar(addr, &threshold, &progress);
  }


  if (exit_bootloader() == NACK)
    hang_error(ERR_GO_FAILED);



  // Teardown
  for (uint8_t i=5;i<40;i+=5) {
    buffer_c[0 + i].high=0;
    buffer_c[1 + i].high=0;
    buffer_c[2 + i].high=0;
    buffer_c[3 + i].high=0;
    buffer_c[4 + i].high=0;
  }
  setDisplayPWM(5);


  return 0;
}

void firmwareCheckOnEject(){
  // device ejected, reboot if new firmware files present

  extern uint32_t _app_crc[];
  uint32_t cur_fwt_crc = _app_crc[0];

  uint32_t new_fwd_crc=0, new_fwt_crc=0;
  uint32_t rc;
  FIL file1, file2;

  if (f_open(&file2, "/FWT.BIN", FA_READ) == FR_OK) {
    f_lseek(&file2, TIME_APP_SIZE - 4);
    f_read(&file2, &new_fwt_crc, 4, &rc);
    if (rc==4 && new_fwt_crc != cur_fwt_crc)
      NVIC_SystemReset();
  }

  if (f_open(&file1, "/FWD.BIN", FA_READ) == FR_OK) {
    f_lseek(&file1, DATE_APP_SIZE - 4);
    f_read(&file1, &new_fwd_crc, 4, &rc);
    if (rc==4 && new_fwd_crc != cur_fwd_crc)
      NVIC_SystemReset();
  }
}
