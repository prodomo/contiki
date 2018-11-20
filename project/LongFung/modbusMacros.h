
/**
 * \brief  Macros used in modbus protocol
 */

//!High part of a value
#define VAL_2_HIGH(val)		(val >> 8)

//!Low part of a value
#define VAL_2_LOW(val)		(val & 0x00FF)
