/*
 * modbus_crc.h
 *
 * Header for CRC-16 calculation functions used in Modbus protocol communication.
 * This header declares the function for computing the CRC-16 checksum as defined
 * in the Modbus protocol standard. The CRC-16 checksum is widely used in Modbus
 * for error-checking in data transmission.
 */

#ifndef INC_MODBUS_CRC_H_
#define INC_MODBUS_CRC_H_

#include <stdint.h>

/**
 * Computes the CRC-16 checksum for a given buffer based on the Modbus protocol standard.
 * The function utilizes a pre-calculated lookup table to efficiently compute the checksum
 * for error detection in Modbus frame data.
 *
 * @param buffer Pointer to the data buffer for which the CRC-16 checksum is to be computed.
 * @param buffer_length Length of the data buffer in bytes.
 * @return The computed CRC-16 checksum as a 16-bit unsigned integer.
 */
uint16_t modbus_crc16(uint8_t *buffer, uint16_t buffer_length);


#endif /* INC_MODBUS_CRC_H_ */

