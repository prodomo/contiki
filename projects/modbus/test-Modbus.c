/**
 * \file
 *         Read Modbus address from Archmeter power meter using Modbus protocols
 * \author
 *         Germ�n Ramos <german.ramos@sensingcontrol.com>
 *         Joakim Eriksson <joakime@sics.se>
 * \company
 *                 Sensing & Control Systems S.L.
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

#include "modbus-api.h"
#include "rs485-dev.h"

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
  int rv = 0;
  PROCESS_BEGIN();

  printf("Starting up tester...\n\r");
  modbus_init();

  while(1) {

    printf("Waiting 1 sec.\n\r");
    etimer_set(&et, 1 * CLOCK_SECOND);

    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    rv = modbus_read_register(0x01, MODBUS_RD_HOLD_REG, 0x0d, 1);
    if(rv == 0) {
	    int data = modbus_get_int(0);
      printf("Success state after sending modbus packet\n\r");
      printf("Value read: %x = %d \n\r", 0x0d, data);
    } else if(rv == -1) {
      printf("Error state after sending modbus packet: -1\n\r");
    } else {
      printf("Error state after sending modbus packet: %d\n\r", rv);
    }
  }
  PROCESS_END();
}