/*
 * fram.c
 *
 * Implements interface functions for the Fujitsu MB85RS64V SPI FRAM module.
 */

#include "fram.h"
#include "spi.h"

/**
 * Enables write operations on the FRAM by sending a Write Enable (WREN) command.
 * @return ERR_OK if the command is successfully transmitted, otherwise the corresponding error code.
 */
static error_t fram_write_enable(void)
{
  error_t status;

  FRAM_CS_LOW();
  status = spi_transmit_single_byte(SPI1, FRAM_CMD_WREN);
  FRAM_CS_HIGH();

  return status;
}

/**
 * Disables write operations on the FRAM by sending a Write Disable (WRDI) command.
 * @return ERR_OK if the command is successfully transmitted, otherwise the corresponding error code.
 */
static error_t fram_write_disable(void)
{
  error_t status;

  FRAM_CS_LOW();
  status = spi_transmit_single_byte(SPI1, FRAM_CMD_WRDI);
  FRAM_CS_HIGH();

  return status;
}

void fram_init(void)
{
  FRAM_CS_HIGH();
  spi_init(spi1);
}

error_t fram_write(uint16_t address, uint8_t *write_buffer, uint16_t data_length)
{
  error_t status = fram_write_enable();

  if (status == ERR_OK)
  {
    FRAM_CS_LOW();

    // Prepare the write command and address
    uint8_t tx_data[FRAM_CMD_AND_ADDRESS_LENGTH] = {0};
    tx_data[0] = FRAM_CMD_WRITE;
    tx_data[1] = (uint8_t)(address >> 8);
    tx_data[2] = (uint8_t)(address);

    if (spi_transmit_bytes(SPI1, tx_data, sizeof(tx_data)) != ERR_OK)
    {
      status = ERR_FAIL;
    }
    else
    {
      // Proceed with writing the data using DMA
      status = spi_transmit_bytes_dma(spi1, write_buffer, data_length);
    }

    FRAM_CS_HIGH();

    status = fram_write_disable();
  }

  return status;
}

error_t fram_read(uint16_t address, uint8_t *read_buffer, uint16_t data_length)
{
  error_t status = ERR_OK;

  FRAM_CS_LOW();

  // Prepare the read command and address
  uint8_t tx_data[FRAM_CMD_AND_ADDRESS_LENGTH] = {0};
  tx_data[0] = FRAM_CMD_READ;
  tx_data[1] = (uint8_t)(address >> 8);
  tx_data[2] = (uint8_t)(address);

  if (spi_transmit_bytes(SPI1, tx_data, sizeof(tx_data)) != ERR_OK)
  {
    status = ERR_FAIL;
  }
  else
  {
    // Proceed with reading the data using DMA
    status = spi_receive_bytes_dma(spi1, read_buffer, data_length);
  }

  FRAM_CS_HIGH();

  return status;
}




