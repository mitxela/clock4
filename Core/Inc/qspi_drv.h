// W25Q128JV

#ifndef QSPI_DRV_H
#define QSPI_DRV_H

#include "stm32l4xx_hal.h"

#define RESET_ENABLE_CMD                     0x66
#define RESET_MEMORY_CMD                     0x99

#define WRITE_ENABLE_CMD                     0x06
#define WRITE_DISABLE_CMD                    0x04

#define BURST_WRAP_CMD                       0x77

#define READ_CMD                             0x03
#define FAST_READ_CMD                        0x0B
#define DUAL_OUT_FAST_READ_CMD               0x3B
#define DUAL_INOUT_FAST_READ_CMD             0xBB
#define QUAD_OUT_FAST_READ_CMD               0x6B
#define QUAD_INOUT_FAST_READ_CMD             0xEB

#define PAGE_PROG_CMD                        0x02
#define QUAD_PAGE_PROG_CMD                   0x32

#define SECTOR_ERASE_CMD                     0x20
#define BLOCK_ERASE_32KB_CMD                 0x52
#define BLOCK_ERASE_64KB_CMD                 0xD8

#define READ_STATUS_REG1_CMD                 0x05
#define READ_STATUS_REG2_CMD                 0x35
#define READ_STATUS_REG3_CMD                 0x15
#define WRITE_STATUS_REG1_CMD                0x01
#define WRITE_STATUS_REG2_CMD                0x31
#define WRITE_STATUS_REG3_CMD                0x11

#define W25Q128_FSR1_BUSY                     (1<<0) /* Reg 1, busy flag */
#define W25Q128_FSR1_WREN                     (1<<1) /* Reg 1, write enable */
#define W25Q128_FSR2_QE                       (1<<1) /* Reg 2, Quad enable */

#define W25Q128_BULK_ERASE_MAX_TIME          250000
#define W25Q128_SECTOR_ERASE_MAX_TIME        3000
#define W25Q128_SUBSECTOR_ERASE_MAX_TIME     800

#define W25Q128_PAGE_SIZE                    0x100
#define W25Q128_SECTOR_SIZE                  0x1000

typedef enum
{
  QSPI_STATUS_ERROR = 0,
  QSPI_STATUS_OK = 1
} QSPI_STATUS;


QSPI_STATUS QSPI_Driver_Init();
QSPI_STATUS QSPI_Read(uint8_t* pData, uint32_t address, uint32_t size);
QSPI_STATUS QSPI_Erase_Sector(uint32_t SectorAddress);
QSPI_STATUS QSPI_Write_Sector(uint8_t *pData, uint32_t address);

uint8_t QSPI_Initialized();
uint8_t QSPI_Locked();

extern uint32_t qspi_write_time;

#endif
