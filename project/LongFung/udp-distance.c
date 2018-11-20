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
// #include "collect-common.h"
#include "collect-view.h"

#include "net/link-stats.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <dev/leds.h>
#include "command-type.h"
#include "net/mac/tsch/tsch.h"
#include "net/mac/tsch/tsch-queue.h"
#include "net/mac/tsch/tsch-private.h"
#include "net/mac/tsch/tsch-log.h"
#include "net/mac/tsch/tsch-packet.h"
#include "net/mac/tsch/tsch-schedule.h"
#include "net/mac/tsch/tsch-slot-operation.h"

#define UDP_CLIENT_PORT 8775
#define UDP_SERVER_PORT 5688

#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"

#include "cc2538-temp-sensor.h"
#include "cfs-coffee-arch.h"
#include "modbus-api.h"
// #include "dev/ain0-sensor.h"

static struct uip_udp_conn *client_conn;
static uip_ipaddr_t server_ipaddr;

static unsigned long time_offset;
static int send_active = 1;
static int send_command = 0;
static int recv_counter =0;
static int ack_flag = 0;
static char* command_data;

#define ACK_PERIOD 1

#define PERIOD 6

#define DEFAULT_DISTANCE 1000
#define DISTANCE_THRESHOLD 650

struct tsch_asn_t start_time;
struct tsch_asn_t end_time;
static uint16_t last_distance = DEFAULT_DISTANCE;
static uint16_t minimun_distance;
static int send_period = PERIOD;
static uint16_t command_id=0;
static int conf_flag=0;
static uint8_t current_state = 0;

/*---------------------------------------------------------------------------*/
PROCESS(udp_client_process, "UDP client process");
AUTOSTART_PROCESSES(&udp_client_process);
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
struct tsch_asn_t
get_timesynch_time(void)
{
  /*---------tsch_asn_t------------------------
    uint32_t ls4b; //least significant 4 bytes
    uint8_t  ms1b; // most significant 1 byte
  ----------------------------------------- */
  // struct tsch_asn_t asn = tsch_current_asn;
  // printf("TSCH: {asn-%x.%lx link-NULL} ", tsch_current_asn.ms1b, tsch_current_asn.ls4b);
  return tsch_current_asn;
}

static unsigned long
get_time(void)
{
  // return clock_seconds() + time_offset;
  return clock_seconds();
}
/*---------------------------------------------------------------------------*/
static unsigned long
strtolong(const char *data) {
  unsigned long value = 0;
  int i;
  for(i = 0; i < 10 && isdigit(data[i]); i++) {
    value = value * 10 + data[i] - '0';
  }
  return value;
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
/*---------------------------------------------------------------------------*/
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
    struct msg{
      uint16_t commandId;
      uint8_t commandType;
    };
    struct msg msg;
    uint8_t sensor_num = 0;

    memset(&msg, 0, sizeof(msg));
    printf("sizeof(msg) %d\n", sizeof(msg));
    appdata = (uint8_t *)uip_appdata;
    printf("--------------------recv data-----------------\n");
    printf("uip_datalen %u\n",uip_datalen());

    memcpy(&msg.commandId, appdata, sizeof(uint16_t));
    appdata+=sizeof(uint16_t);
    memcpy(&msg.commandType, appdata, sizeof(uint8_t));
    appdata+=sizeof(uint8_t);
    printf("msg.commandType %u\n", msg.commandType);
    printf("msg.commandId %u\n", msg.commandId);

    if(uip_datalen()>4)
    {
      memcpy(&sensor_num, appdata, sizeof(uint8_t));
      appdata+=sizeof(uint8_t);
    }

    struct setting_msg setting_msg[sensor_num];

    if(sensor_num>0)
    {
      for(int i=0; i<sensor_num; i++)
      {
        memcpy(&setting_msg[i].setting_type, appdata, sizeof(uint8_t));
        appdata+=sizeof(uint8_t);
        memcpy(&setting_msg[i].sensor_tittle, appdata, sizeof(uint8_t));
        appdata+=sizeof(uint8_t);
        memcpy(&setting_msg[i].value, appdata, sizeof(uint8_t));
        appdata+=sizeof(uint8_t);
      }
    }

    switch(msg.commandType){
      case CMD_TYPE_CONF:
        printf("should send conf\n");
        set_ack_flag(msg.commandId, 1);
        break;
      
      case CMD_TYPE_SET:
        printf("should set value\n");
        set_ack_flag(msg.commandId, 0);
        if(sensor_num>0)
        {
          for(int i=0; i<sensor_num; i++)
          {
            setting_value(setting_msg[i]);
          }
        }
        break;
    }

    leds_toggle(LEDS_RED);
    
    /* Ignore incoming data */
  }
}

/*---------------------------------------------------------------------------*/
void
collect_ack_send(uint16_t commandId)
{
  // printf("generate ack packet\n");
  struct 
  {
    uint16_t command_id;
    uint8_t command_type;
    uint8_t is_received;
  }ack;
  rpl_parent_t *preferred_parent;
  linkaddr_t parent;
  rpl_dag_t *dag;

  if(client_conn == NULL) {
    /* Not setup yet */
    return;
  }
  memset(&ack, 0, sizeof(ack));
  linkaddr_copy(&parent, &linkaddr_null);

  ack.command_id = commandId;
  ack.command_type = CMD_TYPE_ACK;
  ack.is_received = 1;
  // printf("sizeof(ack) %d\n", sizeof(ack));
  printf("ack: %u %u %u\n", ack.command_type, ack.command_id, ack.is_received);

  printf("send ack\n");
  uip_udp_packet_sendto(client_conn, &ack, sizeof(ack),
                        &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));


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
void
collect_common_set_send_active(int active)
{
  send_active = active;
}
/*---------------------------------------------------------------------------*/
void
set_ack_flag(uint16_t commandId, int is_config)
{
  // printf("~~~~~~~~~set_ack_flag\n");
  ack_flag=1;
  conf_flag = is_config;
  command_id = commandId;

}
/*---------------------------------------------------------------------------*/
void
set_send_rate(uint8_t value)
{
  send_period = value;
}
/*---------------------------------------------------------------------------*/
void setting_value(struct setting_msg msg)
{

  if(msg.setting_type == SET_TYPE_RATE && msg.sensor_tittle == SNR_TLE_DEFAULT)
  {
    printf("changing sending rate %u\n", msg.value);
    set_send_rate(msg.value);
  }
  else if(msg.setting_type == SET_TYPE_THRESHOLD)
  {
    switch(msg.sensor_tittle){
      case SNR_TLE_DEFAULT:
        break;
      
      case SNR_TLE_TEMPERATURE:
        printf("changing temperature threshold\n");
        break;
      
      case SNR_TLE_ELE_CURRENT:
        printf("changing electric current threshold\n");
        break;
      
      case SNR_TLE_ROTAT_SPEED:
        printf("changing rotation speed threshold\n");
        break;
      }
  }
  else if(msg.setting_type == SET_TYPE_STATE)
  {
    change_state(msg.value);
    printf("change state!\n");
  }
  else{
    return;
  }

}
/*---------------------------------------------------------------------------*/
void reset_values()
{
  current_state = DEFAULT_STATE;
  minimun_distance = DEFAULT_DISTANCE;
  last_distance = DEFAULT_DISTANCE;
  TSCH_ASN_INIT(start_time,0,0);
  TSCH_ASN_INIT(end_time,0,0);

}
/*---------------------------------------------------------------------------*/
void change_state_send(uint8_t state, struct tsch_asn_t time)
{
  struct data{
    uint16_t command_id;
    uint8_t  command_type;
    uint8_t  change_state;
    uint16_t asn1;
    uint16_t asn2;
  };
  static uint16_t seqno;
  struct data data;

  if(client_conn == NULL) {
    /* Not setup yet */
    return;
  }
  memset(&data, 0, sizeof(data));
  
  seqno++;
  data.command_type= CMD_TYPE_DATA;
  data.command_id = seqno;
  data.change_state = state;
  data.asn1 = (time.ls4b>>16);
  data.asn2 = (uint16_t)time.ls4b;

  printf("change_state_send packet\n");
  printf("{asn-%x.%lx link-NULL} ", time.ms1b, time.ls4b);
  uip_udp_packet_sendto(client_conn, &data, sizeof(data),
                        &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
}
/*---------------------------------------------------------------------------*/
void check_value()
{
  uint16_t current_distance;

  if(current_state == DEFAULT_STATE)
  {
    if(current_distance < last_distance && current_distance < DISTANCE_THRESHOLD)
    {
      if(start_time.ls4b == 0)
      {
        start_time = get_timesynch_time();
      }
      minimun_distance=current_distance;
      last_distance=current_distance;
      change_state_send(SOL_STATE, start_time);
      printf("send packet to Root SOL State\n");
    }
  }
  else if(current_state == SOL_STATE)
  {
    if(current_distance < last_distance && current_distance < minimun_distance)
    {
      last_distance = current_distance;
      minimun_distance = current_distance;
    }
  }
  else if(current_state == MP_STATE)
  {
    if(current_distance > minimun_distance)
    {
      change_state_send(EOL_STATE, end_time);
      end_time = get_timesynch_time();
      printf("send packet to Root EOL STATE\n");
    }
  }
  printf("current_distance %u\n", current_distance);
  printf("last_distance %u\n", last_distance);
}
/*---------------------------------------------------------------------------*/
void
change_state(uint8_t state)
{
  switch(state)
  {
    case SOL_STATE:
      if(current_state == DEFAULT_STATE)
      {
        current_state=SOL_STATE;
      }
      break;
    case PVT_STATE:
      if (current_state == SOL_STATE)
      {
        current_state=PVT_STATE;
      }
      break;
    case MP_STATE:
      if (current_state == PVT_STATE)
      {
        current_state=MP_STATE;
      }
      break;
    case EOL_STATE:
      if (current_state == MP_STATE)
      {
        reset_values();

      }
      break;
    default:
      break;
  }
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_client_process, ev, data)
{
  static struct etimer period_timer, wait_timer, ack_timer, conf_timer;

  PROCESS_BEGIN();

  collect_common_net_init();
  modbus_init();

  /* Send a packet every 60-62 seconds. */
  etimer_set(&period_timer, CLOCK_SECOND * send_period);

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

  while(1) {
    if(ack_flag)
    {
      printf("set ack_timer\n");
      etimer_set(&ack_timer, CLOCK_SECOND * ACK_PERIOD);
    }
    if(conf_flag)
    {
      printf("set conf_timer\n");
      etimer_set(&conf_timer, CLOCK_SECOND * ACK_PERIOD);
    }
    PROCESS_YIELD();
    if(ev == tcpip_event)
    {
      tcpip_handler();
    }else if(ev == PROCESS_EVENT_TIMER)
    {
      if(data == &period_timer){
        etimer_reset(&period_timer);
        check_value();
        // if(send_active)
        // {
        //   /* Time to send the data */
        //   collect_rs485_send(11, 0x4700);
        // }
      }else if(data == &ack_timer && ack_flag)
        {
          printf("ack_timer timeup\n");
          ack_flag=0;
          collect_ack_send(command_id);
        }
        else if(data == &conf_timer && conf_flag)
        {
          printf("conf_timer timeup\n");
          conf_flag=0;
          printf("send config\n");
        }
    }
    else if(ev == serial_line_event_message) {
      char *line;
      line = (char *)data;
      printf("--------------rev command------------:%s\n", line);
      printf("strlen(line) %d\n", strlen(line));
      if(strncmp(line, STARTWORD, 2)==0 && strncmp(line+strlen(line)-3, ENDWORD, 2)==0)
      {
        send_command=1;
        recv_counter++;
        command_data = malloc(strlen(line) + 1);
        strcpy(command_data, line);
        printf("recv_counter: %d\n", recv_counter);
        printf("command_data: %s\n", command_data);
        printf("-----------recv send command-----------\n");
      } else {
        printf("unhandled command: %s\n", line);
      }
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
