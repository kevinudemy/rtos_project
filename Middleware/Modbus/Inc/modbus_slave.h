/*
 * modbus_slave.h
 *
 * This code is inspired by the work of ControllersTech and has been further
 * expanded and developed for practical application in this course.
 */

#ifndef INC_MODBUSSLAVE_H_
#define INC_MODBUSSLAVE_H_

#include "error.h"
#include "modbus_crc.h"

// Slave ID used
#define SLAVE_ID                (7)

// Minimum Modbus RTU message length
#define MODBUS_MIN_MSG_LEN      (4)

// Maximum Modbus RTU message size
#define MODBUS_MAX_MSG_LEN      (256)

// Modbus specifications
#define MODBUS_MAX_NUM_REGS     (125)
#define MODBUS_MAX_NUM_COILS    (2000)

// Exception codes
#define ILLEGAL_FUNCTION        (0x01)
#define ILLEGAL_DATA_ADDRESS    (0x02)
#define ILLEGAL_DATA_VALUE      (0x03)
#define SLAVE_DEVICE_FAILURE    (0x04)

// Reponse length
#define MODBUS_RESPONSE_LENGTH  (6)

/**
 * Modbus Slave receive and transmit buffers.
 * Note: This structure is accessible to the
 * modbus_slave_task.c and the uart.c files.
 */
typedef struct
{
    uint8_t rx_data[MODBUS_MAX_MSG_LEN];
    uint8_t tx_data[MODBUS_MAX_MSG_LEN];
    uint16_t rx_byte_num;
} modbus_buffers_t;
extern modbus_buffers_t modbus_buffers;

// Enum for indices in Modbus messages
typedef enum
{
  SLAVE_ID_IDX = 0,
  FUNC_CODE_IDX = 1,
  // Used for response messages
  BYTE_COUNT_IDX = 2,
  DATA_START_IDX = 3,
  // Used for request messages
  START_ADDR_HIGH_IDX = 2,
  START_ADDR_LOW_IDX = 3,
  NUM_REGS_HIGH_IDX = 4,
  NUM_REGS_LOW_IDX = 5,
  COIL_DATA_START_IDX = 7
} modbus_message_index_e;

/**
 * Prepares a Modbus write response by copying key fields from the received request
 * (slave ID, function code, address, and register values) into the response buffer and sends the response.
 *
 * @param buffers Pointer to the Modbus buffer structure containing request and response data.
 * @return ERR_OK if the response was successfully sent, or an appropriate error code otherwise.
 */
error_t modbus_slave_prepare_and_send_response(modbus_buffers_t *buffers);

/**
 * Prepares and sends a Modbus exception response to the master.
 * The exception response message is structured as:
 *   | SLAVE_ID | FUNCTION_CODE | Exception code | CRC |
 *   | 1 BYTE   | 1 BYTE        | 1 BYTE         | 2 BYTES |
 *
 * The SLAVE_ID and FUNCTION_CODE are obtained from the received request.
 * The MSB of the FUNCTION_CODE is set to indicate an exception.
 * The actual exception code is provided as an argument to the function.
 * The CRC is automatically calculated and appended by the modbus_slave_send_response function.
 *
 * @param exceptioncode The Modbus exception code to send in the response.
 * @return ERR_OK indicating success, or an appropriate error code otherwise.
 */
error_t modbus_slave_exception(uint8_t exception_code);

/**
 * Processes a Modbus request to reads holding registers of the Modbus slave device and
 * prepares a response. The requested register address and count are checked for validity.
 * If valid, the function prepares a Modbus response message with the requested register
 * values. If there's an issue with the request (e.g., invalid register count or address),
 * the function sends a Modbus exception response.
 *
 * The response preparation involves:
 * - Copying the slave ID and function code from the request.
 * - Setting the byte count based on the number of requested registers.
 * - Populating the response with the values of the requested holding registers.
 * - Sending the prepared response using modbus_slave_send_response().
 *
 * @param buffers Pointer to the Modbus buffers structure containing the request
 *                data and where the response will be stored.
 * @return ERR_OK if successful, otherwise an error code if an error occurs.
 */
error_t modbus_slave_read_holding_regs(modbus_buffers_t *buffers);

/**
 * Processes a Modbus request to read input registers of the Modbus slave device and
 * prepares a response. The requested register address and count are checked for validity.
 * If valid, the function prepares a Modbus response message with the requested register
 * values. If there's an issue with the request, an exception response is sent.
 *
 * @param buffers Pointer to the Modbus buffers structure containing the request
 *                data and where the response will be stored.
 * @return ERR_OK if successful, otherwise an error code if an error occurs.
 */
error_t modbus_slave_read_input_regs(modbus_buffers_t *buffers);

/**
 * Processes a Modbus request to read the status of coils by processing the received data,
 * determining the required coils, and then preparing a response with the status of the
 * specified coils. It ensures that the requested number of coils and the starting address
 * are within valid bounds. The function constructs the response based on the status of
 * coils in the Coils Buffer.
 *
 * @param buffers Pointer to the Modbus buffers structure containing the request
 *                data and where the response will be stored.
 * @return ERR_OK if successful, otherwise an error code if an error occurs.
 */
error_t modbus_slave_read_coils(modbus_buffers_t *buffers);

/**
 * Processes a Modbus request to read the status of discrete inputs by processing the received data,
 * determining the required discrete inputs, and then preparing a response with the status of the
 * specified discrete inputs. It ensures that the requested number of input and the starting address
 * are within valid bounds. The function constructs the response based on the status of
 * coils in the Discrete Inputs Buffer.
 *
 * @param buffers Pointer to the Modbus buffers structure containing the request
 *                data and where the response will be stored.
 * @return ERR_OK if successful, otherwise an error code if an error occurs.
 */
error_t modbus_slave_read_discrete_inputs(modbus_buffers_t *buffers);

/**
 * Processes a Modbus request to write values to holding registers of the Modbus slave device.
 * The provided register address, count, and values are checked for validity. If valid,
 * the function updates the holding registers with the given values and stores the updated address
 * and quantity in the output parameters for use by the Modbus data manager.
 *
 * @param buffers Pointer to the Modbus buffers structure containing the request data.
 * @param out_start_addr Pointer to store the starting address of the updated holding registers, for use by the Modbus data manager.
 * @param out_num_regs Pointer to store the number of registers that were updated, for use by the Modbus data manager.
 * @return ERR_OK if successful, otherwise an error code indicating the nature of the error.
 */
error_t modbus_slave_write_holding_regs(modbus_buffers_t *buffers, uint16_t *out_start_addr, uint16_t *out_num_regs);

/**
 * Processes a Modbus request to write to a single holding register of the Modbus slave device.
 * The requested register address is checked for validity. If valid, the function updates
 * the holding register and stores the updated address in the output parameter for use by
 * the Modbus data manager.
 *
 * @param buffers Pointer to the Modbus buffers structure containing the request data.
 * @param out_reg_addr Pointer to store the address of the updated register, for use by the Modbus data manager.
 * @return ERR_OK if successful, otherwise an error code indicating the nature of the error.
 */
error_t modbus_slave_write_single_reg(modbus_buffers_t *buffers, uint16_t *out_reg_addr);

/**
 * Processes a Modbus request to write to a single coil of the Modbus slave device.
 * The requested coil address is checked for validity. If valid, the function updates
 * the coil and stores the updated address in the output parameter for use by
 * the Modbus data manager.
 *
 * @param buffers Pointer to the Modbus buffers structure containing the request data.
 * @param out_coil_addr Pointer to store the address of the updated coil, for use by the Modbus data manager.
 * @return ERR_OK if successful, otherwise an error code indicating the nature of the error.
 */
error_t modbus_slave_write_single_coil(modbus_buffers_t *buffers, uint16_t *out_coil_addr);

/**
 * Processes a Modbus request to write to multiple coils of the Modbus slave device.
 * The requested start address and the number of coils are checked for validity. If valid,
 * the function updates the coils and stores the updated start address and quantity in the output
 * parameters for use by the Modbus data manager.
 *
 * @param buffers Pointer to the Modbus buffers structure containing the request data.
 * @param out_start_addr Pointer to store the starting address of the updated coils, for use by the Modbus data manager.
 * @param out_num_coils Pointer to store the number of coils that were updated, for use by the Modbus data manager.
 * @return ERR_OK if successful, otherwise an error code indicating the nature of the error.
 */
error_t modbus_slave_write_multi_coils(modbus_buffers_t *buffers, uint16_t *out_start_addr, uint16_t *out_num_coils);


#endif /* INC_MODBUSSLAVE_H_ */


