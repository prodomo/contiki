/**
 * \file
 *         Read Modbus address using Modbus protocols
 * \author
 *         Germ�n Ramos <german.ramos@sensingcontrol.com>
 *         Joakim Eriksson <joakime@sics.se>
 *         Jason Huang <jason840507@gmail.com>
 * \company
 *         Industrial Technology Research Institute Taiwan
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

#include "modbus-api.h"

/*---------------------------------------------------------------------------*/
PROCESS(modbus_test_process, "Modbus tester");
AUTOSTART_PROCESSES(&modbus_test_process);
/*---------------------------------------------------------------------------*/
/***********************************************************************/

/**
 * \brief  Modbus main function
 * \param  void
 * \return 0
 */

PROCESS_THREAD(modbus_test_process, ev, data)
{
  static struct etimer et;
  static uint8_t rv = 0;
  static uint8_t i = 1;
  const uint8_t devAddr[2] = {11, 12};
  const int regAddr = 0x4700;
  PROCESS_BEGIN();

  printf("Starting up tester...\n\r");
  modbus_init();

  etimer_set(&et, 3 * CLOCK_SECOND);

  while(1) {
    printf("Waiting 1 sec.\n\r");
    etimer_set(&et, 3 * CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    rv = modbus_read_register(devAddr[i], MODBUS_RD_HOLD_REG, regAddr, 1);
    if(rv == 0) {
	    uint16_t data = modbus_get_int(0);
      printf("Success state after sending modbus packet\n\r");
      printf("Value read form %d: 0x%x = %d \n\r", devAddr[i], regAddr, data);
    } else {
      printf("Error state after sending modbus packet: %d\n\r", rv);
    }

    i = i ^ 1;
  }

  PROCESS_END();
}