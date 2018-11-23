/**
 * \file
 *         Read Modbus address from Archmeter power meter using Modbus protocols
 * \author
 *         Germ�n Ramos <german.ramos@sensingcontrol.com>
 *         Joakim Eriksson <joakime@sics.se>
 *         Jason Huang <jason840507@gmail.com>
 * \company
 *                 Industrial Technology Research Institute Taiwan
 */

/**
 * \brief  This is the main file of modbus protocol
 *
 * We expect:
 *    TX: 0F 04 10 C6 00 02 94 18
 *    RX: 0F 04 04 5A 39 40 1C E6 98
 *
 * This modbus packet will be transmitted over RS485
 */

#include "contiki.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "leds.h"


#include "modbusStructs.h"
#include "modbusProtocol.h"
#include "modbusMacros.h"
#include "rs485-dev.h"

static int modbusAddr = 0;
static int modbusCount = 0;

void init_crc16_tab( void );

/**
 * \brief  It builds query modbus
 * \param  function Function that will be used
 * \param  modbusQuery Query structure where data will be filled to
 * \param  modbusFunctions Pointer to generic structure of all modbus functions
 * \return status of write: 0 sucess, -1 error
 */
static int
modbusRdCoilStatus(unsigned int function,
                   st_modbusIOGeneric *modbusRdCoil,
                   st_modbusQuery *modbusQuery,
                   st_modbusExceptionCode *exceptionCode,
                   unsigned char hasCrc, unsigned int registerModBus)
{
  int status = -1;

  /* Modbus device address */
  modbusQuery->address = modbusAddr;
  /* Function code to read registers(Modbus protocol), 0x04 --> Read Input Registers */
  modbusQuery->function = function;
  modbusQuery->startAddressHI = VAL_2_HIGH(registerModBus);
  modbusQuery->startAddressLO = VAL_2_LOW(registerModBus);
  modbusQuery->countHI = VAL_2_HIGH(modbusCount);
  modbusQuery->countLO = VAL_2_LOW(modbusCount);

  /* TODO: check the zero and set it to what is needed */
  status = modbusReadCoilStatus(modbusQuery, modbusRdCoil,
                                exceptionCode, hasCrc, 0);
  
  return status;
}
/***********************************************************************/

/**
 * \brief  Modbus main function
 * \param  void
 * \return 0
 */
static st_modbusQuery modbusQuery;
static st_modbusIOGeneric modbusStatus;
static st_modbusExceptionCode exceptionCode;

uint8_t
modbus_read_register(unsigned int devAddr,
                     unsigned int function,
                     unsigned int registerAddr,
                     unsigned int regCount)
{
  modbusAddr = devAddr;
  modbusCount = regCount;

  int rv = 0;
  /* Is necessary the use of CRC-16 (Modbus) */
  unsigned char hasCrc = 1;
  
  rv = modbusRdCoilStatus(function, &modbusStatus,
                          &modbusQuery, &exceptionCode, hasCrc,
                          registerAddr);
  watchdog_periodic();
  return rv;
}

st_modbusIOGeneric *modbus_get_status() {
  return &modbusStatus;
}

/********************************************************************/

/* simple function to calculate the float value in modbus */
/* low word high byte | low word low byte || high word ... */
/* Hi - low: SEEE EEEE EMMM MMMM MMMM...MMMM */
float
modbus_get_float(int pos) {
  float f = 0;
  uint32_t mantissa = 0;
  int exp = 0;
  uint8_t *data = modbusStatus.data;

  exp = ((data[pos + 2] & 127) << 1) + (data[pos + 3] >> 7) - 127;
  mantissa = ((uint32_t)(data[pos + 3] & 127) << 16) |
    ((uint32_t)data[pos + 0] << 8) | data[pos + 1];
  f = 1.0f + mantissa / (float)(1L << 23);

/*   printf("Mantissa: %u, exp: %d\n", mantissa, exp); */

  /* really crappy and slow code... */
  while(exp > 0) {
    f = f * 2;
    exp--;
  }
  while(exp < 0) {
    f = f / 2;
    exp++;
  }

  if((data[pos + 2] & 0x80) == 0x80) {
    f = -f;
  }

  return f;
}

uint16_t
modbus_get_int(int pos) {
  uint8_t *data = modbusStatus.data;
  int len = modbusStatus.byteCount;
  int ret = 0;

  for (int i = 0; i < len; i++)
  {
    ret |= data[pos + i] << (8 * (len - i - 1));
  }

  return ret;
}

void
modbus_init()
{
  rs485_init();
  init_crc16_tab();
}
