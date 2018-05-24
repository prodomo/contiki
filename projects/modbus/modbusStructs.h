
/**
 * \brief  This file contais the structs used in modbus protocol
 */

#ifndef __MODBUS_STRUCTS_H__
#define __MODBUS_STRUCTS_H__

#include "modbusDefines.h"


typedef struct {
	unsigned char function;
	unsigned char exceptionCode;
} st_modbusExceptionCode;

typedef struct {
	unsigned char address;
	unsigned char function;
	unsigned char startAddressHI;
	unsigned char startAddressLO;
	unsigned char countHI;
	unsigned char countLO;
} st_modbusQuery;

typedef struct {
	unsigned char function;
	unsigned char byteCount;
    unsigned char data[64]; /* MODBUS_MAX_NUM_COILS]; */
	unsigned char crcHI;
	unsigned char crcLO;
} st_modbusIOGeneric;

#endif /* __MODBUS_STRUCTS_H__ */
