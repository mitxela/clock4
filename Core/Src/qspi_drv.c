#include "qspi_drv.h"
#include <assert.h>

/*volatile uint8_t rx_complete = 0;
volatile uint8_t tx_complete = 0;
volatile uint8_t status_matched = 0;
volatile uint8_t command_complete = 0;
volatile int8_t qspi_locked = 0;
uint8_t qspi_init_state = 0;*/

extern QSPI_HandleTypeDef hqspi;

#include <stdio.h>

static QSPI_STATUS QSPI_ResetMemory();
static QSPI_STATUS QSPI_WriteEnable();
static QSPI_STATUS QSPI_AutoPollingMemReady(uint32_t Timeout);
//static QSPI_STATUS QSPI_Driver_Write_Page(uint8_t *pData, uint32_t address);

/*
uint8_t QSPI_Driver_state()
{
    return qspi_init_state;
}
uint8_t QSPI_Driver_locked()
{
    return qspi_locked;
}
void HAL_QSPI_RxCpltCallback(QSPI_HandleTypeDef *hqspi)
{
    rx_complete = 1;
}
void HAL_QSPI_TxCpltCallback(QSPI_HandleTypeDef *hqspi)
{
    tx_complete = 1;
}
void HAL_QSPI_StatusMatchCallback(QSPI_HandleTypeDef *hqspi)
{
    status_matched = 1;
}
void HAL_QSPI_CmdCpltCallback(QSPI_HandleTypeDef *hqspi)
{
    command_complete = 1;
}
void HAL_QSPI_TimeOutCallback(QSPI_HandleTypeDef *hqspi)
{
    Error_Handler();
}
void HAL_QSPI_ErrorCallback(QSPI_HandleTypeDef *hqspi)
{
    Error_Handler();
}
*/

static QSPI_STATUS QSPI_ResetMemory()
{
  QSPI_CommandTypeDef sCommand;

  /* Initialize the reset enable command */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = RESET_ENABLE_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_NONE;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Send the command */
  if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_STATUS_ERROR;
  }

  /* Send the reset memory command */
  sCommand.Instruction = RESET_MEMORY_CMD;
  if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_STATUS_ERROR;
  }

  /* Configure automatic polling mode to wait the memory is ready */
  if (QSPI_AutoPollingMemReady(HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != QSPI_STATUS_OK)
  {
    return QSPI_STATUS_ERROR;
  }

  return QSPI_STATUS_OK;
}

static QSPI_STATUS QSPI_WriteEnable()
{
  QSPI_CommandTypeDef     sCommand;
  QSPI_AutoPollingTypeDef sConfig;

  /* Enable write operations */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = WRITE_ENABLE_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_NONE;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_STATUS_ERROR;
  }

  /* Configure automatic polling mode to wait for write enabling */
  sConfig.Match           = W25Q128_FSR1_WREN;
  sConfig.Mask            = W25Q128_FSR1_WREN;
  sConfig.MatchMode       = QSPI_MATCH_MODE_AND;
  sConfig.StatusBytesSize = 1;
  sConfig.Interval        = 0x10;
  sConfig.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

  sCommand.Instruction    = READ_STATUS_REG1_CMD;
  sCommand.DataMode       = QSPI_DATA_1_LINE;
  sCommand.NbData         = 1;

  if (HAL_QSPI_AutoPolling(&hqspi, &sCommand, &sConfig, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_STATUS_ERROR;
  }

  return QSPI_STATUS_OK;
}

static QSPI_STATUS QSPI_AutoPollingMemReady(uint32_t Timeout)
{
  QSPI_CommandTypeDef     sCommand;
  QSPI_AutoPollingTypeDef sConfig;

  /* Configure automatic polling mode to wait for memory ready */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = READ_STATUS_REG1_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_1_LINE;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  sConfig.Match           = 0x00;
  sConfig.Mask            = W25Q128_FSR1_BUSY;
  sConfig.MatchMode       = QSPI_MATCH_MODE_AND;
  sConfig.StatusBytesSize = 1;
  sConfig.Interval        = 0x10;
  sConfig.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

  if (HAL_QSPI_AutoPolling(&hqspi, &sCommand, &sConfig, Timeout) != HAL_OK)
  {
    return QSPI_STATUS_ERROR;
  }

  return QSPI_STATUS_OK;
}

QSPI_STATUS QSPI_Driver_Init()
{
  QSPI_CommandTypeDef sCommand;
  uint8_t value = W25Q128_FSR2_QE;

  /* QSPI memory reset */
  if (QSPI_ResetMemory() != QSPI_STATUS_OK)
  {
    return QSPI_STATUS_ERROR;
  }

  /* Enable write operations */
  if (QSPI_WriteEnable() != QSPI_STATUS_OK)
  {
    return QSPI_STATUS_ERROR;
  }

  /* Set status register for Quad Enable */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = WRITE_STATUS_REG2_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_1_LINE;
  sCommand.DummyCycles       = 0;
  sCommand.NbData            = 1;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_STATUS_ERROR;
  }
  /* Transmit the data */
  if (HAL_QSPI_Transmit(&hqspi, &value, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_STATUS_ERROR;
  }

  /* automatic polling mode to wait for memory ready */
  if (QSPI_AutoPollingMemReady(W25Q128_SUBSECTOR_ERASE_MAX_TIME) != QSPI_STATUS_OK)
  {
    return QSPI_STATUS_ERROR;
  }

  return QSPI_STATUS_OK;
}

QSPI_STATUS QSPI_Driver_Read(uint8_t* pData, uint32_t address, uint32_t size)
{
  QSPI_CommandTypeDef sCommand;

  /* Reading Sequence ------------------------------------------------ */
  sCommand.NbData      = size;
  sCommand.Address     = address;
  sCommand.Instruction = QUAD_INOUT_FAST_READ_CMD;
  sCommand.DummyCycles = DUMMY_CLOCK_CYCLES_READ_QUAD;
  sCommand.DataMode    = QSPI_DATA_4_LINES;

  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.AddressMode       = QSPI_ADDRESS_4_LINES;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_STATUS_ERROR;
  }

  /* Reception of the data */
  if (HAL_QSPI_Receive(&hqspi, pData, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_STATUS_ERROR;
  }

  return QSPI_STATUS_OK;
}

QSPI_STATUS QSPI_Erase_Sector(uint32_t SectorAddress)
{
  QSPI_CommandTypeDef sCommand;

  /* Initialize the erase command */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = SECTOR_ERASE_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_1_LINE;
  sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
  sCommand.Address           = SectorAddress;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_NONE;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Enable write operations */
  if (QSPI_WriteEnable() != QSPI_STATUS_OK)
  {
    return QSPI_STATUS_ERROR;
  }

  /* Send the command */
  if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_STATUS_ERROR;
  }

  /* Configure automatic polling mode to wait for end of erase */
  if (QSPI_AutoPollingMemReady(W25Q128_SUBSECTOR_ERASE_MAX_TIME) != QSPI_STATUS_OK)
  {
    return QSPI_STATUS_ERROR;
  }

  return QSPI_STATUS_OK;
}

QSPI_STATUS QSPI_Driver_Write_Page(uint8_t *pData, uint32_t address)
{
  QSPI_CommandTypeDef sCommand;

  /* Enable write operations */
  if (QSPI_WriteEnable() != QSPI_STATUS_OK)
  {
    return QSPI_STATUS_ERROR;
  }
  //QSPI_GetStatus();

  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = QUAD_PAGE_PROG_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_1_LINE;
  sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_4_LINES;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  sCommand.Address = address;
  sCommand.NbData  = W25Q128_PAGE_SIZE;

  /* Configure the command */
  if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_STATUS_ERROR;
  }

  /* Transmission of the data */
  if (HAL_QSPI_Transmit(&hqspi, pData, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_STATUS_ERROR;
  }

  //QSPI_GetStatus();

  /* Configure automatic polling mode to wait for end of program */
  if (QSPI_AutoPollingMemReady(HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != QSPI_STATUS_OK)
  {
    return QSPI_STATUS_ERROR;
  }
  //QSPI_GetStatus();
  return QSPI_STATUS_OK;
}

QSPI_STATUS QSPI_Driver_Write_Sector(uint8_t *pData, uint32_t address)
{
  uint32_t written = 0;
  do {
    if (QSPI_Driver_Write_Page(&pData[written], address) != QSPI_STATUS_OK)
    {
      return QSPI_STATUS_ERROR;
    }
    address += W25Q128_PAGE_SIZE;
    written += W25Q128_PAGE_SIZE;
  } while (written < W25Q128_SECTOR_SIZE);

  return QSPI_STATUS_OK;
}



uint8_t QSPI_GetStatus(void)
{
  QSPI_CommandTypeDef sCommand;
  uint8_t reg;

  /* Initialize the read flag status register command */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = READ_STATUS_REG1_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_1_LINE;
  sCommand.DummyCycles       = 0;
  sCommand.NbData            = 1;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_STATUS_ERROR;
  }

  /* Reception of the data */
  if (HAL_QSPI_Receive(&hqspi, &reg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_STATUS_ERROR;
  }

  printf("Status: 0x%02X\n", reg);

  return reg;
  /* Check the value of the register */
  /*if((reg & W25Q128FV_FSR_BUSY) != 0)
  {
    return QSPI_BUSY;
  }
  else
  {
    return QSPI_STATUS_OK;
  }*/
}


QSPI_STATUS BSP_QSPI_Write(uint8_t* pData, uint32_t WriteAddr, uint32_t Size)
{
  QSPI_CommandTypeDef sCommand;
  uint32_t end_addr, current_size, current_addr;

  /* Calculation of the size between the write address and the end of the page */
  current_addr = 0;

  while (current_addr <= WriteAddr)
  {
    current_addr += W25Q128_PAGE_SIZE;
  }
  current_size = current_addr - WriteAddr;

  /* Check if the size of the data is less than the remaining place in the page */
  if (current_size > Size)
  {
    current_size = Size;
  }

  /* Initialize the address variables */
  current_addr = WriteAddr;
  end_addr = WriteAddr + Size;

  /* Initialize the program command */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = QUAD_PAGE_PROG_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_1_LINE;
  sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_4_LINES;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Perform the write page by page */
  do
  {
    sCommand.Address = current_addr;
    sCommand.NbData  = current_size;

    /* Enable write operations */
    if (QSPI_WriteEnable() != QSPI_STATUS_OK)
    {
      return QSPI_STATUS_ERROR;
    }

    /* Configure the command */
    if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      return QSPI_STATUS_ERROR;
    }

    /* Transmission of the data */
    if (HAL_QSPI_Transmit(&hqspi, pData, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      return QSPI_STATUS_ERROR;
    }

    /* Configure automatic polling mode to wait for end of program */
    if (QSPI_AutoPollingMemReady(HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != QSPI_STATUS_OK)
    {
      return QSPI_STATUS_ERROR;
    }

    /* Update the address and size variables for next page programming */
    current_addr += current_size;
    pData += current_size;
    current_size = ((current_addr + W25Q128_PAGE_SIZE) > end_addr) ? (end_addr - current_addr) : W25Q128_PAGE_SIZE;
  } while (current_addr < end_addr);

  return QSPI_STATUS_OK;
}



QSPI_STATUS QSPI_Driver_Read_Single(uint8_t* pData, uint32_t ReadAddr, uint32_t size)
{
  QSPI_CommandTypeDef sCommand;

  /* Reading Sequence ------------------------------------------------ */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = READ_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_1_LINE;
  sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
  sCommand.Address           = ReadAddr;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_1_LINE;
  sCommand.DummyCycles       = 0;
  sCommand.NbData            = size;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_STATUS_ERROR;
  }

  /* Reception of the data */
  if (HAL_QSPI_Receive(&hqspi, pData, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_STATUS_ERROR;
  }

  return QSPI_STATUS_OK;
}
