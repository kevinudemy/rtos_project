/*
 * error.h
 *
 * Contains the type definition for error_t.
 */

#ifndef INC_ERROR_H_
#define INC_ERROR_H_

/**
 * Return type to be used for functions that require an error status returned.
 */
typedef int error_t;

/**
 * Enumeration of error constants (to be expanded as required).
 */
typedef enum
{
  ERR_OK = 0,
  ERR_FAIL = -1,
  ERR_TIMEOUT = -2,
  ERR_INVALID_PARAM = -3
} error_e;

#endif /* INC_ERROR_H_ */
