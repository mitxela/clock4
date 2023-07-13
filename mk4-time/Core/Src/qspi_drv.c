#include "qspi_drv.h"

extern QSPI_HandleTypeDef hqspi;


static QSPI_STATUS QSPI_ResetMemory();
static QSPI_STATUS QSPI_WriteEnable();
static QSPI_STATUS QSPI_AutoPollingMemReady(uint32_t Timeout);
static QSPI_STATUS QSPI_Write_Page(uint8_t *pData, uint32_t address);

uint8_t initialized = 0;
uint8_t locked = 0;
uint32_t qspi_write_time = 0;

uint8_t QSPI_Initialized()
{
  return initialized;
}
uint8_t QSPI_Locked()
{
  return locked;
}

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

  locked++;

  /* QSPI memory reset */
  if (QSPI_ResetMemory() != QSPI_STATUS_OK)
  {
    locked--;
    return QSPI_STATUS_ERROR;
  }

  /* Enable write operations */
  if (QSPI_WriteEnable() != QSPI_STATUS_OK)
  {
    locked--;
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
    locked--;
    return QSPI_STATUS_ERROR;
  }
  /* Transmit the data */
  if (HAL_QSPI_Transmit(&hqspi, &value, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    locked--;
    return QSPI_STATUS_ERROR;
  }

  /* automatic polling mode to wait for memory ready */
  if (QSPI_AutoPollingMemReady(W25Q128_SUBSECTOR_ERASE_MAX_TIME) != QSPI_STATUS_OK)
  {
    locked--;
    return QSPI_STATUS_ERROR;
  }

  initialized = 1;
  locked--;
  return QSPI_STATUS_OK;
}

QSPI_STATUS QSPI_Erase_Sector(uint32_t SectorAddress)
{
  QSPI_CommandTypeDef sCommand;

  locked++;

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
    locked--;
    return QSPI_STATUS_ERROR;
  }

  /* Send the command */
  if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    locked--;
    return QSPI_STATUS_ERROR;
  }

  /* Configure automatic polling mode to wait for end of erase */
  if (QSPI_AutoPollingMemReady(W25Q128_SUBSECTOR_ERASE_MAX_TIME) != QSPI_STATUS_OK)
  {
    locked--;
    return QSPI_STATUS_ERROR;
  }

  locked--;
  return QSPI_STATUS_OK;
}

QSPI_STATUS QSPI_Write_Page(uint8_t *pData, uint32_t address)
{
  QSPI_CommandTypeDef sCommand;

  locked++;

  /* Enable write operations */
  if (QSPI_WriteEnable() != QSPI_STATUS_OK)
  {
    locked--;
    return QSPI_STATUS_ERROR;
  }

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
    locked--;
    return QSPI_STATUS_ERROR;
  }

  /* Transmission of the data */
  if (HAL_QSPI_Transmit(&hqspi, pData, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    locked--;
    return QSPI_STATUS_ERROR;
  }

  /* Configure automatic polling mode to wait for end of program */
  if (QSPI_AutoPollingMemReady(HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != QSPI_STATUS_OK)
  {
    locked--;
    return QSPI_STATUS_ERROR;
  }

  locked--;
  return QSPI_STATUS_OK;
}

QSPI_STATUS QSPI_Write_Sector(uint8_t *pData, uint32_t address)
{
  uint32_t written = 0;
  do {
    if (QSPI_Write_Page(&pData[written], address) != QSPI_STATUS_OK)
    {
      return QSPI_STATUS_ERROR;
    }
    address += W25Q128_PAGE_SIZE;
    written += W25Q128_PAGE_SIZE;
  } while (written < W25Q128_SECTOR_SIZE);

  return QSPI_STATUS_OK;
}


QSPI_STATUS QSPI_Read(uint8_t* pData, uint32_t ReadAddr, uint32_t size)
{
  QSPI_CommandTypeDef sCommand;

  if (size==0) return QSPI_STATUS_OK;

  locked++;
  if (locked>1) {
    locked--;
    return QSPI_STATUS_LOCKED;
  }

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
    locked--;
    return QSPI_STATUS_ERROR;
  }

  /* Reception of the data */
  if (HAL_QSPI_Receive(&hqspi, pData, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    locked--;
    return QSPI_STATUS_ERROR;
  }

  locked--;
  return QSPI_STATUS_OK;
}

