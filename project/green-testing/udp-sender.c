/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

#include "contiki.h"
#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ip/uip-udp-packet.h"
#include "net/rpl/rpl.h"
#include "dev/serial-line.h"
#if CONTIKI_TARGET_Z1
#include "dev/uart0.h"
#else
#include "dev/uart1.h"
#endif
#include "collect-common.h"
#include "collect-view.h"

#include "net/link-stats.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <dev/leds.h>
#include "command-type.h"

#define UDP_CLIENT_PORT 8775
#define UDP_SERVER_PORT 5688

#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"

#define LEDS_MY_GREEN 128
#define BEEP_PERIOD 1

#include "cc2538-temp-sensor.h"
#include "cfs-coffee-arch.h"
// #include "dev/ain0-sensor.h"

static struct uip_udp_conn *client_conn;
static uip_ipaddr_t server_ipaddr;
volatile int set_beep_on =0;

static temp_value=0;
static int ain0_value=0, ain1_value=0;
static uint8_t my_link_join=0;
int temperature_threshold=50;
#define MIN_TEMPERATURE -20
#define MAX_TEMPERATURE 120

/*---------------------------------------------------------------------------*/
PROCESS(udp_client_process, "UDP client process");
AUTOSTART_PROCESSES(&udp_client_process, &collect_common_process);
/*---------------------------------------------------------------------------*/
void
collect_common_set_sink(void)
{
  /* A udp client can never become sink */
}
/*---------------------------------------------------------------------------*/
int
change_TemperatureValue_to_RealValue(int value)
{
  // 13428=50c, 23428=120c

  int result = 0;
  int zero = 628550;
  int one = 14285;

  value = value*100;

  if(value>=zero)
  {
    result= (value-zero)/one;
    printf("%d to result: %d \n",value, result);
    return result;
  }
  else
  {
    result = 0-((zero-value)/one);
    printf("%d to result: %d \n",value, result);
    return result;
  }

}
/*---------------------------------------------------------------------------*/
void set_temperature_threshold(int value)
{
  printf("orignal threshold %d\n", temperature_threshold);
  temperature_threshold = value;
  printf("after change %d\n", temperature_threshold);
}
/*---------------------------------------------------------------------------*/

unsigned int
uart1_send_bytes(const unsigned char *s, unsigned int len)
{
  unsigned int i = 0;

  while(s && *s != 0) {
    if(i >= len) {
      break;
    }
    uart_write_byte(1, *s++);
    i++;
  }
  return i;
}

void
collect_common_net_print(void)
{
  rpl_dag_t *dag;
  uip_ds6_route_t *r;

  /* Let's suppose we have only one instance */
  dag = rpl_get_any_dag();
  if(dag->preferred_parent != NULL) {
    PRINTF("Preferred parent: ");
    PRINT6ADDR(rpl_get_parent_ipaddr(dag->preferred_parent));
    PRINTF("\n");
  }
  for(r = uip_ds6_route_head();
      r != NULL;
      r = uip_ds6_route_next(r)) {
    PRINT6ADDR(&r->ipaddr);
  }
  PRINTF("---\n");
}
/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{
  if(uip_newdata()) {
    uint8_t *appdata;
    uint8_t data;
    struct command_msg msg;
    memset(&msg, 0, sizeof(msg));
    appdata = (uint8_t *)uip_appdata;
    printf("--------------------recv data-----------------\n");
    printf("uip_datalen %u\n",uip_datalen());
    memcpy(&msg.type, appdata, sizeof(data));
    appdata+=sizeof(data);
    memcpy(&msg.value, appdata, sizeof(data));

    printf("msg %u %u\n", msg.type, msg.value);
    leds_toggle(LEDS_RED);
    switch(msg.type)
    {
      case RATE_TYPE:
      {
        set_send_rate(msg.value);
        break;
      }
      case BEEP_TYPE:
      {
        if(msg.value == 0)
        {
          printf("set beep off\n");
          set_beep_on = 0;
          // leds_off(LEDS_ORANGE);
        }
        else if(msg.value ==1)
        {
          printf("set beep on\n");
          set_beep_on = 1;
        }
        break;
      }
      case THRESHOLD_TYPE:
      {
        printf("set threshold\n");
        set_temperature_threshold(msg.value);
      }
      default:
        break;
    }

    /* Ignore incoming data */
  }
}
void
collect_special_send(char* data)
{
  /* Server never sends */
}
/*---------------------------------------------------------------------------*/
void
collect_common_send(void)
{
  static uint8_t seqno;
  char flash[8];
  struct {
    uint8_t seqno;
    uint8_t for_alignment;
    struct collect_view_data_msg msg;
  } msg;
  /* struct collect_neighbor *n; */
  uint16_t parent_etx;
  uint16_t rtmetric;
  uint16_t num_neighbors;
  uint16_t beacon_interval;
  int16_t  parent_rssi;
  rpl_parent_t *preferred_parent;
  linkaddr_t parent;
  rpl_dag_t *dag;
  // static uint16_t count=0;
  // char string[20];
  // int temp_value;
  // int ain0_value, ain1_value;

  if(client_conn == NULL) {
    /* Not setup yet */
    return;
  }
  memset(&msg, 0, sizeof(msg));
  seqno++;
  if(seqno == 0) {
    /* Wrap to 128 to identify restarts */
    seqno = 128;
  }
  msg.seqno = seqno;

  linkaddr_copy(&parent, &linkaddr_null);
  parent_etx = 0;

  /* Let's suppose we have only one instance */
  dag = rpl_get_any_dag();
  if(dag != NULL) {
    my_link_join = dag->joined;
    preferred_parent = dag->preferred_parent;
    if(preferred_parent != NULL) {
      uip_ds6_nbr_t *nbr;
      nbr = uip_ds6_nbr_lookup(rpl_get_parent_ipaddr(preferred_parent));
      if(nbr != NULL) {
        /* Use parts of the IPv6 address as the parent address, in reversed byte order. */
        parent.u8[LINKADDR_SIZE - 1] = nbr->ipaddr.u8[sizeof(uip_ipaddr_t) - 2];
        parent.u8[LINKADDR_SIZE - 2] = nbr->ipaddr.u8[sizeof(uip_ipaddr_t) - 1];
        parent_etx = rpl_get_parent_rank((uip_lladdr_t *) uip_ds6_nbr_get_ll(nbr)) / 2;

        //get parent rssi
        parent_rssi = rpl_get_parent_link_stats(preferred_parent)->rssi;
      }
    }
    rtmetric = dag->rank;
    beacon_interval = (uint16_t) ((2L << dag->instance->dio_intcurrent) / 1000);
    num_neighbors = uip_ds6_nbr_num();
  } else {
    my_link_join = 0;
    rtmetric = 0;
    beacon_interval = 0;
    num_neighbors = 0;
  }

  // msg.msg.sensors[0]=ain0_value; //external-temperature0
  // msg.msg.sensors[1]=ain1_value; //external-temperature1
  // msg.msg.sensors[2]=temp_value; //inter-temperature

  /* num_neighbors = collect_neighbor_list_num(&tc.neighbor_list); */
  collect_view_construct_message(&msg.msg, &parent,
                                 parent_etx, rtmetric,
                                 num_neighbors, beacon_interval);

  msg.msg.sensors[5]=parent_rssi;

  struct tsch_asn_t time;
  time = get_timesynch_time();
  msg.msg.sensors[7]=(time.ls4b>>16);
  msg.msg.sensors[8]=(uint16_t)time.ls4b;
  printf("asn-%lx",time.ls4b);
  printf("msg[7 %lx .8 %lx]\n", (time.ls4b>>16), (uint16_t)time.ls4b);

  cfs_offset_t offset;
  offset = 0x00050000;
  // for(int i=0; i<8 ; i++)
  //   flash[i] = i;
  // COFFEE_WRITE(flash, sizeof(flash), offset);
  // printf ("write flash");

  COFFEE_READ_1(flash, sizeof(flash), offset);
  printf("flash: \n");
  for(int i=0; i<8 ; i++)
    printf("%x\n", flash[i]);

  printf("send packet\n");
  uip_udp_packet_sendto(client_conn, &msg, sizeof(msg),
                        &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));

  // my test
  //leds_arch_set(7);
  //leds_toggle(LEDS_MY_GREEN);
  //leds_toggle(LEDS_YELLOW);
  // leds_toggle(LEDS_GREEN);
  //fade(LEDS_YELLOW);
  //fade(LEDS_GREEN);
  //sprintf(string, "sending string %u.\n", ++count);
  //uart1_send_bytes((uint8_t *)string, sizeof(string) - 1);
  //printf("Printf, neig=%d, parentetx=%d, TEMP=%d \n", num_neighbors, parent_etx, ALS_SENSOR);
  // ain0_value=msg.msg.sensors[0];
  // ain1_value=msg.msg.sensors[1];
  // temp_value=msg.msg.sensors[2];

  // printf("Printf, neig=%d, parentetx=%d, ain0=%d,ain1=%d value=%d \n", num_neighbors, parent_etx, ain0_value, ain1_value, temp_value);
  if(dag != NULL)
  {
    printf("dag->joined: %d\n", dag->joined);
  }
  else
  {
    printf("dag is NULL\n");
  }
}
/*---------------------------------------------------------------------------*/
void
collect_common_net_init(void)
{
#if CONTIKI_TARGET_Z1
  uart0_set_input(serial_line_input_byte);
#else
  uart1_set_input(serial_line_input_byte);
#endif
  serial_line_init();
}
/*---------------------------------------------------------------------------*/
static void
print_local_addresses(void)
{
  int i;
  uint8_t state;

  PRINTF("Client IPv6 addresses: ");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(uip_ds6_if.addr_list[i].isused &&
       (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
      PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
      PRINTF("\n");
      /* hack to make address "final" */
      if (state == ADDR_TENTATIVE) {
        uip_ds6_if.addr_list[i].state = ADDR_PREFERRED;
      }
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
set_global_address(void)
{
  uip_ipaddr_t ipaddr;

  uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);

  /* set server address */
  uip_ip6addr(&server_ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 1);

}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_client_process, ev, data)
{
  static struct etimer beep_timer;

  PROCESS_BEGIN();

  PROCESS_PAUSE();

  set_global_address();

  PRINTF("UDP client process started\n");

  print_local_addresses();
  NETSTACK_MAC.on();
  /* new connection with remote host */
  client_conn = udp_new(NULL, UIP_HTONS(UDP_SERVER_PORT), NULL);
  udp_bind(client_conn, UIP_HTONS(UDP_CLIENT_PORT));

  PRINTF("Created a connection with the server ");
  PRINT6ADDR(&client_conn->ripaddr);
  PRINTF(" local/remote port %u/%u\n",
        UIP_HTONS(client_conn->lport), UIP_HTONS(client_conn->rport));

  etimer_set(&beep_timer, CLOCK_SECOND * BEEP_PERIOD / 5 );

  while(1) {
        PROCESS_YIELD();
    if(ev == tcpip_event) {
      tcpip_handler();
    }else if(ev == PROCESS_EVENT_TIMER){
      // 13428=50c, 23428=120c
      int temperature = change_TemperatureValue_to_RealValue(ain1_value);
      if((temperature>temperature_threshold) && (temperature<MAX_TEMPERATURE))
        set_beep_on=1;
      else{
        set_beep_on=0;
        // leds_off(LEDS_ORANGE);
      }
      
      if(data == &beep_timer){
        //decide link LEDs
        if(my_link_join == 1)
          leds_toggle(LEDS_GREEN);
        else
          leds_on(LEDS_GREEN);

        //beep alarm
        if(set_beep_on)
          // leds_toggle(LEDS_ORANGE);
        etimer_reset(&beep_timer);
      }
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
