/*
 * modbus_slave.c
 *
 * Implements Modbus RTU slave functionality, including function codes and response sending.
 *
 * Originally inspired by the work of ControllersTech, this implementation
 * has been expanded and customized for practical use in this course.
 */

#include <string.h>

#include "modbus_slave.h"
#include "modbus_regs.h"
#include "error_handler_task.h"
#include "uart.h"

/*
 * Initialization of Modbus register buffers.
 */
uint8_t modbus_coils[(COIL_MAX + 7) / 8]            = {0};
uint8_t modbus_discrete_inputs[(DISC_MAX + 7) / 8]  = {0};
uint16_t modbus_input_regs[INPUT_MAX]               = {0};
uint16_t modbus_holding_regs[HOLDING_MAX]           = {0};

/**
 * Prepares a Modbus response message by appending the CRC to the
 * end of the provided data and transmits the complete message over UART.
 * @param data data buffer containing the Modbus message without the CRC.
 * @param size Size of the data in the buffer without the CRC.
 * @return ERR_OK if successful, otherwise an error.
 */
static error_t modbus_slave_send_response(uint8_t *data, int size)
{
  // Calculate the CRC
  uint16_t crc = modbus_crc16(data, size);
  data[size] = (crc & 0xFF);          // CRC LOW
  data[size+1] = ((crc >> 8) & 0xFF); // CRC HIGH

  error_t send_status = uart_transmit_bytes(USART2, data, size + sizeof(crc));

  // Handle UART transmission errors
  if (send_status != ERR_OK)
  {
    error_handler_send_msg(EVT_MODBUS_UART_TX_ERROR);  // Send an error message
  }

  return send_status;
}

error_t modbus_slave_prepare_and_send_response(modbus_buffers_t *buffers)
{
    // Prepare Response
    buffers->tx_data[SLAVE_ID_IDX] = SLAVE_ID;
    buffers->tx_data[FUNC_CODE_IDX] = buffers->rx_data[FUNC_CODE_IDX];
    buffers->tx_data[START_ADDR_HIGH_IDX] = buffers->rx_data[START_ADDR_HIGH_IDX];
    buffers->tx_data[START_ADDR_LOW_IDX] = buffers->rx_data[START_ADDR_LOW_IDX];
    buffers->tx_data[NUM_REGS_HIGH_IDX] = buffers->rx_data[NUM_REGS_HIGH_IDX];
    buffers->tx_data[NUM_REGS_LOW_IDX] = buffers->rx_data[NUM_REGS_LOW_IDX];

    // Send Response
    return modbus_slave_send_response(buffers->tx_data, MODBUS_RESPONSE_LENGTH);
}

error_t modbus_slave_exception(uint8_t exception_code)
{
  modbus_buffers.tx_data[SLAVE_ID_IDX]   = modbus_buffers.rx_data[SLAVE_ID_IDX];          // Slave ID
  modbus_buffers.tx_data[FUNC_CODE_IDX]  = modbus_buffers.rx_data[FUNC_CODE_IDX] | 0x80;  // Add 1 to the MSB of the function code
  modbus_buffers.tx_data[BYTE_COUNT_IDX] = exception_code;                                // Load the Exception code
  return modbus_slave_send_response(modbus_buffers.tx_data, DATA_START_IDX);              // Send Data... CRC will be calculated in the function
}

error_t modbus_slave_read_holding_regs(modbus_buffers_t *buffers)
{
  const uint8_t BYTE_MULTIPLIER = 2;  // For Modbus holding registers

  uint16_t start_addr = ((buffers->rx_data[START_ADDR_HIGH_IDX] << 8) | buffers->rx_data[START_ADDR_LOW_IDX]);
  uint16_t num_regs = ((buffers->rx_data[NUM_REGS_HIGH_IDX] << 8) | buffers->rx_data[NUM_REGS_LOW_IDX]);

  if ((num_regs < 1) || (num_regs > MODBUS_MAX_NUM_REGS))
  {
    modbus_slave_exception(ILLEGAL_DATA_VALUE);
    return MODBUS_INVALID_REG_COUNT;
  }

  uint16_t end_addr = start_addr + num_regs - 1;
  if (end_addr >= HOLDING_MAX)
  {
    modbus_slave_exception(ILLEGAL_DATA_ADDRESS);
    return MODBUS_INVALID_END_ADDRESS;
  }

  // Prepare tx_data buffer
  buffers->tx_data[SLAVE_ID_IDX] = SLAVE_ID;
  buffers->tx_data[FUNC_CODE_IDX] = buffers->rx_data[FUNC_CODE_IDX];
  buffers->tx_data[BYTE_COUNT_IDX] = num_regs * BYTE_MULTIPLIER;

  int index = DATA_START_IDX;
  for (int i = 0; i < num_regs; i++)
  {
    buffers->tx_data[index++] = (modbus_holding_regs[start_addr] >> 8) & 0xFF;
    buffers->tx_data[index++] = (modbus_holding_regs[start_addr]) & 0xFF;
    start_addr++;
  }

  return modbus_slave_send_response(buffers->tx_data, index);
}

error_t modbus_slave_read_input_regs(modbus_buffers_t *buffers)
{
  const uint8_t BITS_PER_BYTE = 8;
  const uint8_t BYTE_MULTIPLIER = 2;  // For Modbus input registers

  uint16_t start_addr = ((buffers->rx_data[START_ADDR_HIGH_IDX] << BITS_PER_BYTE) | buffers->rx_data[START_ADDR_LOW_IDX]);
  uint16_t num_regs = ((buffers->rx_data[NUM_REGS_HIGH_IDX] << BITS_PER_BYTE) | buffers->rx_data[NUM_REGS_LOW_IDX]);

  if ((num_regs < 1) || (num_regs > MODBUS_MAX_NUM_REGS))
  {
    modbus_slave_exception(ILLEGAL_DATA_VALUE);
    return MODBUS_INVALID_REG_COUNT;
  }

  uint16_t end_addr = start_addr + num_regs - 1;
  if (end_addr >= INPUT_MAX)
  {
    modbus_slave_exception(ILLEGAL_DATA_ADDRESS);
    return MODBUS_INVALID_END_ADDRESS;
  }

  // Prepare tx_data buffer
  buffers->tx_data[SLAVE_ID_IDX] = SLAVE_ID;
  buffers->tx_data[FUNC_CODE_IDX] = buffers->rx_data[FUNC_CODE_IDX];
  buffers->tx_data[BYTE_COUNT_IDX] = num_regs * BYTE_MULTIPLIER;

  int index = DATA_START_IDX;
  for (int i = 0; i < num_regs; i++)
  {
    buffers->tx_data[index++] = (modbus_input_regs[start_addr] >> BITS_PER_BYTE) & 0xFF;
    buffers->tx_data[index++] = (modbus_input_regs[start_addr]) & 0xFF;
    start_addr++;
  }

  return modbus_slave_send_response(buffers->tx_data, index);
}

error_t modbus_slave_read_coils(modbus_buffers_t *buffers)
{
  const uint8_t BITS_PER_BYTE = 8;

  uint16_t start_addr = ((buffers->rx_data[START_ADDR_HIGH_IDX] << BITS_PER_BYTE) | buffers->rx_data[START_ADDR_LOW_IDX]);
  uint16_t num_coils = ((buffers->rx_data[NUM_REGS_HIGH_IDX] << BITS_PER_BYTE) | buffers->rx_data[NUM_REGS_LOW_IDX]);

  if ((num_coils < 1) || (num_coils > MODBUS_MAX_NUM_COILS))
  {
    modbus_slave_exception(ILLEGAL_DATA_VALUE);
    return MODBUS_INVALID_COIL_COUNT;
  }

  uint16_t end_addr = start_addr + num_coils - 1;
  if (end_addr >= COIL_MAX)
  {
    modbus_slave_exception(ILLEGAL_DATA_ADDRESS);
    return MODBUS_INVALID_END_ADDRESS;
  }

  // Reset the tx_data
  memset(buffers->tx_data, '\0', MODBUS_MAX_MSG_LEN);

  // Prepare TxData buffer
  buffers->tx_data[SLAVE_ID_IDX] = SLAVE_ID;
  buffers->tx_data[FUNC_CODE_IDX] = buffers->rx_data[FUNC_CODE_IDX];
  buffers->tx_data[BYTE_COUNT_IDX] = (num_coils / BITS_PER_BYTE) + ((num_coils % BITS_PER_BYTE) > 0 ? 1 : 0);
  int index = DATA_START_IDX;

  int start_byte = start_addr / BITS_PER_BYTE;
  uint16_t bit_position = start_addr % BITS_PER_BYTE;
  int index_position = 0;

  for (int i = 0; i < num_coils; i++)
  {
    buffers->tx_data[index] |= ((modbus_coils[start_byte] >> bit_position) & 0x01) << index_position;
    index_position++;
    bit_position++;
    if (index_position > 7)
    {
      index_position = 0;
      index++;
    }
    if (bit_position > 7)
    {
      bit_position = 0;
      start_byte++;
    }
  }

  if (num_coils % BITS_PER_BYTE != 0)
  {
    index++;
  }

  return modbus_slave_send_response(buffers->tx_data, index);
}

error_t modbus_slave_read_discrete_inputs(modbus_buffers_t *buffers)
{
  const uint8_t BITS_PER_BYTE = 8;

  uint16_t start_addr = ((buffers->rx_data[START_ADDR_HIGH_IDX] << BITS_PER_BYTE) | buffers->rx_data[START_ADDR_LOW_IDX]);
  uint16_t num_discrete_inputs = ((buffers->rx_data[NUM_REGS_HIGH_IDX] << BITS_PER_BYTE) | buffers->rx_data[NUM_REGS_LOW_IDX]);

  if ((num_discrete_inputs < 1) || (num_discrete_inputs > MODBUS_MAX_NUM_COILS))
  {
    modbus_slave_exception(ILLEGAL_DATA_VALUE);
    return MODBUS_INVALID_DISC_COUNT;
  }

  uint16_t endAddr = start_addr + num_discrete_inputs - 1;
  if (endAddr >= DISC_MAX)
  {
    modbus_slave_exception(ILLEGAL_DATA_ADDRESS);
    return MODBUS_INVALID_END_ADDRESS;
  }

  // Reset the tx_data
  memset(buffers->tx_data, '\0', MODBUS_MAX_MSG_LEN);

  // Prepare TxData buffer
  buffers->tx_data[SLAVE_ID_IDX]   =  SLAVE_ID;
  buffers->tx_data[FUNC_CODE_IDX]  = buffers->rx_data[1];
  buffers->tx_data[BYTE_COUNT_IDX] = (num_discrete_inputs / BITS_PER_BYTE) + ((num_discrete_inputs % BITS_PER_BYTE) > 0 ? 1 : 0);
  int index = DATA_START_IDX;

  int start_byte = start_addr / BITS_PER_BYTE;
  uint16_t bit_position = start_addr % BITS_PER_BYTE;
  int index_position = 0;

  for (int i = 0; i < num_discrete_inputs; i++)
  {
    buffers->tx_data[index] |= ((modbus_discrete_inputs[start_byte] >> bit_position) & 0x01) << index_position;
    index_position++;
    bit_position++;
    if (index_position > 7)
    {
      index_position = 0;
      index++;
    }
    if (bit_position > 7)
    {
      bit_position = 0;
      start_byte++;
    }
  }

  if (num_discrete_inputs % BITS_PER_BYTE != 0)
  {
    index++;
  }

  return modbus_slave_send_response(buffers->tx_data, index);
}

error_t modbus_slave_write_holding_regs(modbus_buffers_t *buffers,
                                        uint16_t *out_start_addr,
                                        uint16_t *out_num_regs)
{
  const uint8_t BITS_PER_BYTE = 8;

  uint16_t start_addr = ((buffers->rx_data[START_ADDR_HIGH_IDX] << BITS_PER_BYTE) | buffers->rx_data[START_ADDR_LOW_IDX]);
  uint16_t num_regs = ((buffers->rx_data[NUM_REGS_HIGH_IDX] << BITS_PER_BYTE) | buffers->rx_data[NUM_REGS_LOW_IDX]);

  if ((num_regs < 1) || (num_regs > MODBUS_MAX_NUM_REGS)) // constraint as per the Modbus specification
  {
      modbus_slave_exception(ILLEGAL_DATA_VALUE);
      return MODBUS_INVALID_REG_COUNT;
  }

  uint16_t end_addr = start_addr + num_regs - 1;
  if (end_addr >= HOLDING_MAX)                            // constraint based on the number of registers you're supporting
  {
      modbus_slave_exception(ILLEGAL_DATA_ADDRESS);
      return MODBUS_INVALID_END_ADDRESS;
  }

  int index = DATA_START_IDX + 4;           // Adjusted index based on Modbus write holding register function format
  uint16_t initial_start_addr = start_addr; // store the initial start address before writing
  for (int i = 0; i < num_regs; i++)
  {
      modbus_holding_regs[start_addr] = (buffers->rx_data[index] << 8) | buffers->rx_data[index + 1];
      start_addr++;
      index += 2;
  }

  // Update the out params so that the task knows what's changed
  if (out_start_addr) // NULL check
  {
    *out_start_addr = initial_start_addr;  // use the initial start address
  }
  if (out_num_regs)   // NULL check
  {
    *out_num_regs = num_regs;
  }

  return ERR_OK;
}

error_t modbus_slave_write_single_reg(modbus_buffers_t *buffers, uint16_t *out_reg_addr)
{
  const uint8_t BITS_PER_BYTE = 8;

  uint16_t reg_addr = (buffers->rx_data[START_ADDR_HIGH_IDX] << BITS_PER_BYTE) | buffers->rx_data[START_ADDR_LOW_IDX];

  if (reg_addr >= HOLDING_MAX)  // Check for valid register address
  {
    modbus_slave_exception(ILLEGAL_DATA_ADDRESS);
    return MODBUS_INVALID_REG_ADDRESS;
  }

  // Save the 16 bit data from the request message
  modbus_holding_regs[reg_addr] = (buffers->rx_data[NUM_REGS_HIGH_IDX] << BITS_PER_BYTE) | buffers->rx_data[NUM_REGS_LOW_IDX];

  // Update the out parameter
  if (out_reg_addr) // NULL check
  {
    *out_reg_addr = reg_addr;
  }

  return ERR_OK;
}

error_t modbus_slave_write_single_coil(modbus_buffers_t *buffers, uint16_t *out_coil_addr)
{
  const uint8_t BITS_PER_BYTE = 8;

  uint16_t coil_addr = (buffers->rx_data[START_ADDR_HIGH_IDX] << BITS_PER_BYTE) | buffers->rx_data[START_ADDR_LOW_IDX];

  if (coil_addr >= COIL_MAX)  // Check for valid coil address
  {
    modbus_slave_exception(ILLEGAL_DATA_ADDRESS);
    return MODBUS_INVALID_COIL_ADDRESS;
  }

  // Calculation for the bit in the database, where the modification will be done
  int start_byte = coil_addr / BITS_PER_BYTE;           // Determine the byte containing the coil
  uint16_t bit_position = coil_addr % BITS_PER_BYTE;    // Determine the bit position within that byte

  // The next 2 bytes in the request message determine the state of the coil
  if ((buffers->rx_data[NUM_REGS_HIGH_IDX] == 0xFF) && (buffers->rx_data[NUM_REGS_LOW_IDX] == 0x00))
  {
    modbus_coils[start_byte] |= 1 << bit_position;    // Set the coil bit to 1
  }
  else if ((buffers->rx_data[NUM_REGS_HIGH_IDX] == 0x00) && (buffers->rx_data[NUM_REGS_LOW_IDX] == 0x00))
  {
    modbus_coils[start_byte] &= ~(1 << bit_position); // Set the coil bit to 0
  }
  else
  {
    modbus_slave_exception(ILLEGAL_DATA_VALUE);
    return MODBUS_INVALID_COIL_VALUE;
  }

  if (out_coil_addr)  // NULL check
  {
    *out_coil_addr = coil_addr;
  }

  return ERR_OK;
}

error_t modbus_slave_write_multi_coils(modbus_buffers_t *buffers, uint16_t *out_start_addr, uint16_t *out_num_coils)
{
  const uint8_t BITS_PER_BYTE = 8;

  uint16_t start_addr = (buffers->rx_data[START_ADDR_HIGH_IDX] << BITS_PER_BYTE) | buffers->rx_data[START_ADDR_LOW_IDX];
  uint16_t num_coils = (buffers->rx_data[NUM_REGS_HIGH_IDX] << BITS_PER_BYTE) | buffers->rx_data[NUM_REGS_LOW_IDX];

  if (num_coils < 1 || num_coils > MODBUS_MAX_NUM_COILS)  // As per the Modbus Specification
  {
    modbus_slave_exception(ILLEGAL_DATA_VALUE);
    return MODBUS_INVALID_COIL_COUNT;
  }

  uint16_t end_addr = start_addr + num_coils - 1;
  if (end_addr >= COIL_MAX)
  {
    modbus_slave_exception(ILLEGAL_DATA_ADDRESS);
    return MODBUS_INVALID_END_ADDRESS;
  }

  int start_byte = start_addr / BITS_PER_BYTE;
  uint16_t bit_position = start_addr % BITS_PER_BYTE;
  int index_position = 0;

  int index = COIL_DATA_START_IDX;

  for (int i = 0; i < num_coils; i++)
  {
    if (((buffers->rx_data[index] >> index_position) & 0x01) == 1)
    {
      modbus_coils[start_byte] |= 1 << bit_position;
    }
    else
    {
      modbus_coils[start_byte] &= ~(1 << bit_position);
    }

    bit_position++;
    index_position++;

    if (index_position > 7)
    {
      index_position = 0;
      index++;
    }
    if (bit_position > 7)
    {
      bit_position = 0;
      start_byte++;
    }
  }

  // Update the out parameters
  if (out_start_addr) // NULL check
  {
    *out_start_addr = start_addr;
  }
  if (out_num_coils) // NULL check
  {
    *out_num_coils = num_coils;
  }

  return ERR_OK;
}




