/****************************************************/
/* \brief This file contains the functions responsible to implement
   the MODBUS protocol.
*/
/****************************************************/
#include "contiki.h"
#include <stdio.h>
#include "modbusStructs.h"
#include "modbusProtocol.h"
#include "modbusMacros.h"
#include "modbusDefines.h"
#include "rs485-dev.h"

#define DEBUG 0

/****************************************************/
/* Table needed to build CRC-16 */
static unsigned short crc_tab16[256];
/****************************************************/

/*******************************************************************\
*                                                                   *
*   static void init_crc16_tab( void );                             *
*                                                                   *
*   The function init_crc16_tab() is used  to  fill  the  array     *
*   for calculation of the CRC-16 with values.                      *
*                                                                   *
\*******************************************************************/

void
init_crc16_tab( void ) {

  int i, j;
  unsigned short crc, c;
  
  for(i=0; i<256; i++) {
    crc = 0;
    c = (unsigned short) i;
    
    for(j=0; j<8; j++) {
      if( (crc ^ c) & 0x0001 ) crc = ( crc >> 1 ) ^ P_16;
      else crc =   crc >> 1;
      c = c >> 1;
    }
    crc_tab16[i] = crc;
  }
}

static unsigned int
modbusCRC(unsigned char *data, int dataLen)
{
  unsigned int crc = 0xffff, i;
  unsigned short tmp, short_c;
  
  for(i = 0; i < dataLen; i++) {
    short_c = 0x00FF & (unsigned short) data[i];
    tmp = crc ^ short_c;
    crc = (crc >> 8) ^ crc_tab16[ tmp & 0xFF];
  }

  return crc;
}
/****************************************************/
int modbusSendQuery(unsigned char *data, unsigned char dataLen,
		    unsigned char hasCRC)
{
  unsigned int crc;
  int sendStatus;

  if(hasCRC) {
    crc = modbusCRC(data, dataLen);
    data[dataLen + 1] = VAL_2_HIGH(crc);
    data[dataLen] = VAL_2_LOW(crc);
    dataLen += MODBUS_CRC_LENGTH;
  }

/*   printf("TX data (%d):", dataLen); */
/*   for(i = 0; i < dataLen; i++) { */
/*     printf("%02x", data[i]); */
/*   } */
/*   printf("\n\r"); */
  
  clock_wait(2);

  /* reset input buffer before transmit */
  rs485_input_reset();
  sendStatus = rs485_transmit(data, dataLen);

  /* wait some time for reception... */
  clock_wait(5);

  return sendStatus;
}
/****************************************************/
int modbusReadResponse(unsigned char *data, unsigned char hasCRC)
{
  int i;
  int len;
  int numberOfDataBytes;

  if((len = rs485_input_len()) <= 0) {
      printf("RX Error...length <= 0\n\r"); 
    return -1;
  }

  printf("Read data (%d):", len);
  for(i = 0; i < len; i++) {
    printf("%02x", rs485_buffer[i]);
  }
  printf("\n\r");


  /* Address of Modbus device */
  data[0] = rs485_buffer[0];
  
  if(data[0] > MODBUS_EXC_ADD) { /* Error! */
    //Comented printf("### Modbus error EXE_ADD???\n\r");
    return -1;
  }
  
  /* Modbus function */
  data[1] = rs485_buffer[1];

  /* Number of data bytes */
  data[2] = rs485_buffer[2];
  numberOfDataBytes = data[2];

  if(numberOfDataBytes > len) {
    //Comented printf("### Modbus error too long size\n\r");
    return -1;
  }
  /* now we read the data */
  for(i = 3; i < 3 + numberOfDataBytes; i++) {
    data[i] = rs485_buffer[i];
  }

  if (hasCRC) {
    data[3 + numberOfDataBytes] = rs485_buffer[3 + numberOfDataBytes];
    data[3 + numberOfDataBytes + 1] = rs485_buffer[3 + numberOfDataBytes + 1];
  }

  return numberOfDataBytes;
}
/****************************************************/
int modbusRespErrorsVerify(unsigned char *responsePacket, int byteCount, st_modbusExceptionCode *exceptionCode, unsigned char hasCrc)
{
  unsigned int crcTest;

  if( (byteCount < 0) && (responsePacket[0] > MODBUS_EXC_ADD) ){ //Error during read
    //Comented //Comented printf("Error Reading serial!\n\r");
    return -1;
  }

  exceptionCode->function = 0xff;
  exceptionCode->exceptionCode = 0xff;

  if( (byteCount < 0) && (responsePacket[0] > MODBUS_EXC_ADD) ){
    /* Exception Error */
    exceptionCode->function = responsePacket[0] - MODBUS_EXC_ADD;
    exceptionCode->exceptionCode = responsePacket[1];
    return -1;
  }

  //CRC test
  if(hasCrc) {
    /* all data except CRC */
    int dataLen = byteCount + MODBUS_HEADER_LENGTH + 1;
    crcTest = modbusCRC(responsePacket, dataLen);
    if( ( (responsePacket[dataLen + 1]) != (VAL_2_HIGH(crcTest)) )
	|| ( (responsePacket[dataLen]) != (VAL_2_LOW(crcTest)) ) ) {
      return -1;
    }
  }
  
  return 0;
}
/****************************************************/
/* allocate some extra just to be sure... */
static unsigned char responsePacket[64];

int modbusReadCoilStatus(st_modbusQuery *modbusQuery, 
  st_modbusIOGeneric *coilStatus, st_modbusExceptionCode *exceptionCode,
  unsigned char hasCrc, unsigned int registerModBus)
{
  /* allocate some bytes for the data packet */
  uint8_t queryPacket[6 + 2 + 5];
  int byteCount, hasError, i;
  int len;
  
  byteCount = 0;

  /* build the packet into modbus query format */
  queryPacket[0] = modbusQuery->address;
  queryPacket[1] = modbusQuery->function;
  queryPacket[2] = modbusQuery->startAddressHI; //high order
  queryPacket[3] = modbusQuery->startAddressLO; //low order
  queryPacket[4] = modbusQuery->countHI;
  queryPacket[5] = modbusQuery->countLO;
  len = 6;

/*   printf("Reading register %x \n\r", */
/* 	 (modbusQuery->startAddressHI << 8) + */
/* 	 (modbusQuery->startAddressLO)); */
  
  /* send the modbus packet */
  if (modbusSendQuery(queryPacket, len, hasCrc) < 0)
    return -1;

  clock_wait(8);
  //clock_delay(0xFFFF);

  /* read the response */
  byteCount = modbusReadResponse(responsePacket, hasCrc);
  
  /* printf("byteCount: %d\n\r", byteCount); */
  if (byteCount <= 0) {
    printf("Error during reading: no data?\n\r");
    return byteCount;
  }
  
  /* Error Check */
  hasError = modbusRespErrorsVerify(responsePacket, byteCount, exceptionCode, hasCrc);
  printf("CRC error %d \n\r", hasError);
  
  if(!hasError) {
    coilStatus->function = responsePacket[1];
    coilStatus->byteCount = responsePacket[2];
    for (i = MODBUS_HEADER_LENGTH + 1; i < MODBUS_HEADER_LENGTH + 1 + byteCount; i++)
      coilStatus->data[i - (MODBUS_HEADER_LENGTH + 1)] = responsePacket[i];
    if(hasCrc) {
      coilStatus->crcLO = responsePacket[byteCount + MODBUS_HEADER_LENGTH + 1];
      coilStatus->crcHI = responsePacket[byteCount + MODBUS_HEADER_LENGTH + 2];
    }
  }

  return hasError;
}
/****************************************************/
