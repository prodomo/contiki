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
#include "dev/sensor-gpio.h"
#include "dev/sensor-gpio2.h"
// #include "dev/button-sensor.h"

static struct uip_udp_conn *client_conn;
static uip_ipaddr_t server_ipaddr;

static unsigned long time_offset;
static int send_active = 1;
static int send_command = 0;
static int recv_counter =0;
static int ack_flag = 0;
static char* command_data;

#define ACK_PERIOD 1
#define PERIOD 1
#define BUFFSIZE 5

#define DEFAULT_DISTANCE 1000
#define MODBUS_NOTREAD  9999
#define DISTANCE_THRESHOLD 10
#define DISTANCE_ERROR 3


static int send_period = PERIOD;
static int distance_threshold = DISTANCE_THRESHOLD;

struct tsch_asn_t start_time, end_time, open_time, close_time , sensor_upper_time, sensor_downer_time;
// rtimer_clock_t sensor_upper_time=0, sensor_downer_time =0;

static uint16_t last_distance = DEFAULT_DISTANCE;
static uint16_t minimun_distance = DEFAULT_DISTANCE;
static uint16_t maximun_distance = 0;
static uint16_t command_id=0;
static int conf_flag=0;

static uint16_t current_state = 0;
static uint16_t sub_state = 0, send_state=0;  
static uint16_t amount_counter = 0;           //production amount counter
static uint16_t counter = 0;                  //Order counter
static int32_t last_asn_diff;
static int same_counter = 0;                  //asn_diff same counter
static int buff_counter=0;
static int gpio_counter=0;

static uint16_t distance_buff[BUFFSIZE];
static uint16_t sub_state_buff[BUFFSIZE];
static uint16_t temperature_v_buff[BUFFSIZE];
static uint16_t total_v_buff[BUFFSIZE];
static uint16_t temperature_a_buff[BUFFSIZE];
static uint16_t total_a_buff[BUFFSIZE];
static struct gpio_log gpio_buff[BUFFSIZE];



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
      uint16_t commandType;
    };
    struct msg msg;
    uint16_t sensor_num = 0;

    memset(&msg, 0, sizeof(msg));
    printf("sizeof(msg) %d\n", sizeof(msg));
    appdata = (uint8_t *)uip_appdata;
    printf("--------------------recv data-----------------\n");
    printf("uip_datalen %u\n",uip_datalen());

    memcpy(&msg.commandId, appdata, sizeof(uint16_t));
    appdata+=sizeof(uint16_t);
    memcpy(&msg.commandType, appdata, sizeof(uint16_t));
    appdata+=sizeof(uint16_t);
    printf("msg.commandType %u\n", msg.commandType);
    printf("msg.commandId %u\n", msg.commandId);

    if(uip_datalen()>4)
    {
      memcpy(&sensor_num, appdata, sizeof(uint16_t));
      appdata+=sizeof(uint16_t);
    }

    struct setting_msg setting_msg[sensor_num];

    if(sensor_num>0)
    {
      for(int i=0; i<sensor_num; i++)
      {
        memcpy(&setting_msg[i].setting_type, appdata, sizeof(uint16_t));
        appdata+=sizeof(uint16_t);
        memcpy(&setting_msg[i].sensor_tittle, appdata, sizeof(uint16_t));
        appdata+=sizeof(uint16_t);
        memcpy(&setting_msg[i].value, appdata, sizeof(uint16_t));
        appdata+=sizeof(uint16_t);
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
    uint16_t command_type;
    uint16_t is_received;
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

  memset(&distance_buff, 0, sizeof(uint16_t)*BUFFSIZE);
  memset(&sub_state_buff, 0, sizeof(uint16_t)*BUFFSIZE);
  memset(&total_v_buff, 0, sizeof(uint16_t)*BUFFSIZE);
  memset(&temperature_v_buff, 0, sizeof(uint16_t)*BUFFSIZE);
  memset(&total_a_buff, 0, sizeof(uint16_t)*BUFFSIZE);
  memset(&temperature_a_buff, 0, sizeof(uint16_t)*BUFFSIZE);
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

void set_distance_threshold(uint8_t value)
{
  distance_threshold = value;
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
      case SNR_TLE_DISTANCE:
        set_distance_threshold(msg.value);
        printf("changing distance threshold\n");
        break;
      }
  }
  else if(msg.setting_type == SET_TYPE_STATE)
  {
    change_state(msg.value);
    printf("change state! value=%d \n", msg.value);
  }
  else{
    return;
  }

}
/*---------------------------------------------------------------------------*/
void reset_values()
{
  current_state = DEFAULT_STATE;
  sub_state = DEFAULT_STATE;
  send_state = DEFAULT_STATE;

  amount_counter = 0;           //production amount counter
  same_counter = 0;                  //asn_diff same counter
  buff_counter = 0;

  last_distance = DEFAULT_DISTANCE;
  minimun_distance = DEFAULT_DISTANCE;

  // collect_common_set_send_active(0);
  TSCH_ASN_INIT(start_time, 0, 0);
  TSCH_ASN_INIT(end_time, 0, 0);
  TSCH_ASN_INIT(open_time, 0, 0);
  TSCH_ASN_INIT(close_time, 0, 0);
  TSCH_ASN_INIT(sensor_upper_time, 0, 0);
  TSCH_ASN_INIT(sensor_downer_time, 0, 0);

  // sensor_upper_time=0;
  // sensor_downer_time =0;

  memset(&distance_buff, 0, sizeof(uint16_t)*BUFFSIZE);
  memset(&sub_state_buff, 0, sizeof(uint16_t)*BUFFSIZE);
  memset(&total_v_buff, 0, sizeof(uint16_t)*BUFFSIZE);
  memset(&temperature_v_buff, 0, sizeof(uint16_t)*BUFFSIZE);
  memset(&total_a_buff, 0, sizeof(uint16_t)*BUFFSIZE);
  memset(&temperature_a_buff, 0, sizeof(uint16_t)*BUFFSIZE);

  counter++;

}
/*---------------------------------------------------------------------------*/
void change_state_send(uint16_t state)
{
  struct tsch_asn_t current_time;
  struct data{
    uint16_t command_id;
    uint16_t command_type;
    uint16_t change_state;
    uint16_t counter;
    uint16_t amount_counter;
    uint16_t sub_state[BUFFSIZE];
    uint16_t distance[BUFFSIZE];
    uint16_t total_v[BUFFSIZE];
    uint16_t temp_v [BUFFSIZE];
    uint16_t total_a[BUFFSIZE];
    uint16_t temp_a [BUFFSIZE];
  };
  static uint16_t seqno;
  struct data data;

  current_time=get_timesynch_time();
  if(client_conn == NULL) {
    /* Not setup yet */
    return;
  }
  memset(&data, 0, sizeof(data));
  
  seqno++;
  data.command_type= CMD_TYPE_DATA;  //0
  data.command_id = seqno;
  data.change_state = state;
  data.counter = counter;
  data.amount_counter = amount_counter;
  for(int i=0; i<BUFFSIZE; i++)
  {
    data.sub_state[i] = sub_state_buff[i];
    data.distance[i] = distance_buff[i];
    data.total_v[i] = total_v_buff[i];  
    data.temp_v[i] = temperature_v_buff[i]; 
    data.total_a[i] = total_a_buff[i];
    data.temp_a[i] = temperature_a_buff[i];
  }
  buff_counter=0;
  // printf("change_state_send packet\n");
  // printf("state %d counter %d \n",data.change_state, data.counter);
  // printf("sub_state %u distance %u total_v %u temp_v %u \n", data.sub_state[0], data.distance[0], data.total_v[0], data.temp_v[0]);
  uip_udp_packet_sendto(client_conn, &data, sizeof(data),
                        &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
}
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
void
gpio_log_send()
{
  // printf("generate ack packet\n");
  struct data
  {
    uint16_t command_id;
    uint16_t command_type;
    struct gpio_log gpio_send[BUFFSIZE];
  };

  static uint16_t seqno;
  struct data data;

  if(client_conn == NULL) {
    /* Not setup yet */
    return;
  }
  memset(&data, 0, sizeof(data));
  
  seqno++;
  data.command_type= CMD_TYPE_DATA;  //0
  data.command_id = seqno;
  for(int i=0; i<BUFFSIZE; i++)
  {
    data.gpio_send[i].state=gpio_buff[i].state;
    data.gpio_send[i].gpio=gpio_buff[i].gpio;
  }

  // printf("sizeof(ack) %d\n", sizeof(ack));
  // printf("data: %u %u %u\n", ack.command_type, ack.command_id, ack.is_received);

  uip_udp_packet_sendto(client_conn, &data, sizeof(data),
                        &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));


}
/*---------------------------------------------------------------------------*/


void check_v_value()
{
  uint16_t current_total_v = MODBUS_NOTREAD;
  uint16_t current_tempature_v = MODBUS_NOTREAD;

  uint8_t rv = modbus_read_register(2, MODBUS_RD_HOLD_REG, 0x0135, 1);
  if(rv == 0) {
    current_total_v = modbus_get_int(0);
    // printf("Success state after sending modbus packet\n\r");
    printf("Value read form %d: 0x%x = %d \n", 2, 0x0135, current_total_v);
  // } else {
    // printf("Error state after sending modbus packet: %d\n", rv);
  }

  rv = modbus_read_register(3, MODBUS_RD_HOLD_REG, 0x0135, 1);
  if(rv == 0) {
    current_tempature_v = modbus_get_int(0);
    // printf("Success state after sending modbus packet\n\r");
    printf("Value read form %d: 0x%x = %d \n", 3, 0x0135, current_tempature_v);
  // } else {
    // printf("Error state after sending modbus packet: %d\n", rv);
  }

  total_v_buff[buff_counter]=current_total_v;
  temperature_v_buff[buff_counter]=current_tempature_v;
}
/*---------------------------------------------------------------------------*/
void check_a_value()
{
  uint16_t current_total_a = MODBUS_NOTREAD;
  uint16_t current_tempature_a = MODBUS_NOTREAD;

  uint8_t rv = modbus_read_register(2, MODBUS_RD_HOLD_REG, 0x013D, 1);
  if(rv == 0) {
    current_total_a = modbus_get_int(0);
    // printf("Success state after sending modbus packet\n\r");
    printf("Value read form %d: 0x%x = %d \n", 2, 0x013D, current_total_a);
  // } else {
    // printf("Error state after sending modbus packet: %d\n", rv);
  }

  rv = modbus_read_register(3, MODBUS_RD_HOLD_REG, 0x013D, 1);
  if(rv == 0) {
    current_tempature_a = modbus_get_int(0);
    // printf("Success state after sending modbus packet\n\r");
    printf("Value read form %d: 0x%x = %d \n", 3, 0x013D, current_tempature_a);
  // } else {
    // printf("Error state after sending modbus packet: %d\n", rv);
  }

  total_a_buff[buff_counter]=current_total_a;
  temperature_a_buff[buff_counter]=current_tempature_a;
}
/*---------------------------------------------------------------------------*/
void check_distance_value()
{
  uint16_t current_distance;

  uint8_t rv = modbus_read_register(1, MODBUS_RD_HOLD_REG, 0x0082, 1);
  if(rv == 0) {
    current_distance = modbus_get_int(0);
    // printf("Success state after sending modbus packet\n");
    printf("Value read form %d: 0x%x = %d \n", 1, 0x0082, current_distance);
    
    if(last_distance != DEFAULT_DISTANCE && current_distance!=0)
    {
      // printf("last_distance-current_distance %d\n", last_distance-current_distance);
      if((last_distance-current_distance) >= distance_threshold) //current < last
      {
        sub_state = START_CLOSE;
        
        // close_time=get_timesynch_time();
        
        if(current_distance< minimun_distance)
        {
          minimun_distance = current_distance;
        }
        
        if(send_state <= SOL_STATE)
        {
          send_state = PVT_STATE;
          // printf("send_state change to PVT_STATE\n");
        }
      }
      else if(abs(last_distance-current_distance) <= DISTANCE_ERROR && sub_state == START_CLOSE)
      {
        // printf("last_distance == current_distance\n");
        sub_state=CLOSE;
        // close_time=get_timesynch_time();
      }
      else if((current_distance-minimun_distance)>=distance_threshold && sub_state == CLOSE)
      {
        // open_time=get_timesynch_time();
        sub_state=START_OPEN;
        // printf("last_asn_diff %u\n", last_asn_diff);
        // printf("this_asn_diff %u\n", TSCH_ASN_DIFF(open_time,close_time));
        // if(abs(last_asn_diff-TSCH_ASN_DIFF(open_time,close_time))<=101)
        // {
        //   same_counter++;
        // }
        // else
        // {
        //   same_counter=0;
        // }
        amount_counter++;
        // printf("same_counter %d\n", same_counter);
        // last_asn_diff = TSCH_ASN_DIFF(open_time,close_time);
      }
      else if(abs(last_distance-current_distance) <= DISTANCE_ERROR && sub_state == START_OPEN )
      {
        sub_state=OPEN;
      }
      else if(abs(last_distance-current_distance) <= DISTANCE_ERROR && sub_state == OPEN)
      {
        // if(same_counter>2 && send_state<MP_STATE)
        // {
        //   send_state = MP_STATE;
        //   same_counter = 0;
        // }
        // else if(amount_counter >20 && send_state<MP_STATE)
        // {
        //   send_state = MP_STATE;
        // }
        minimun_distance = DEFAULT_DISTANCE;
        
        if(current_distance > maximun_distance)
        {
          maximun_distance = current_distance;
        }
      }
    }
    if(current_distance != 0)
    {
      last_distance = current_distance;
    }
  } else {
    // printf("Error state after sending modbus packet: %d\n", rv);
    current_distance=MODBUS_NOTREAD;
  }

  sub_state_buff[buff_counter] = sub_state;
  distance_buff[buff_counter] = current_distance;
  // printf("sub_state %u distance_buff %u \n", sub_state, current_distance);
}
/*---------------------------------------------------------------------------*/
void
check_photoelectric_sensors()
{
  // send_state = SOL_STATE;
  // printf("check sensor\n");

  if(send_state == SOL_STATE)
  {
    // sensor_upper_time=0;
    // sensor_downer_time=0;
    TSCH_ASN_INIT(sensor_upper_time, 0, 0);
    TSCH_ASN_INIT(sensor_downer_time, 0, 0);

    return;
  }

  if(sensor_upper_time.ms1b==0 || sensor_downer_time.ms1b==0)
  {
    return;
  }

  // printf("sensor_upper_time %u\n", sensor_upper_time);
  // printf("sensor_downer_time %u\n", sensor_downer_time);

  if(abs((int)TSCH_ASN_DIFF(sensor_upper_time, sensor_downer_time))>=101)
  {
    if((int)(sensor_downer_time.ls4b-sensor_upper_time.ls4b)>0 && send_state==DEFAULT_STATE)
    {
      send_state = SOL_STATE;
      // printf("change send_state to SOL_STATE\n");
      // collect_common_set_send_active(1);
      start_time = get_timesynch_time();
      printf("down, in \n");
      // sensor_upper_time=0;
      // sensor_downer_time=0;
      TSCH_ASN_INIT(sensor_upper_time, 0, 0);
      TSCH_ASN_INIT(sensor_downer_time, 0, 0);
    }
    else if((int)(sensor_downer_time.ls4b-sensor_upper_time.ls4b)<0 && send_state==PVT_STATE)
    {
      send_state = EOL_STATE;
      // printf("change send_state to EOL_STATE\n");
      // collect_common_set_send_active(1);
      end_time = get_timesynch_time();
      printf("up, out \n");
      // sensor_upper_time=0;
      // sensor_downer_time=0;
      TSCH_ASN_INIT(sensor_upper_time, 0, 0);
      TSCH_ASN_INIT(sensor_downer_time, 0, 0);
      // reset_values();
    }
  }
  else{
    // sensor_upper_time=0;
    // sensor_downer_time=0;
    TSCH_ASN_INIT(sensor_upper_time, 0, 0);
    TSCH_ASN_INIT(sensor_downer_time, 0, 0);
  }
}
/*---------------------------------------------------------------------------*/
void 
check_value(){
  check_distance_value();
  check_v_value();
  check_a_value();
  buff_counter++;
}
/*---------------------------------------------------------------------------*/
void
change_state(uint16_t state)
{
  // if(send_state == state)
  // {
  //   printf("active=0\n");
  //   collect_common_set_send_active(0);
  // }
  switch(state)
  {
    case SOL_STATE:
      reset_values();
      if(current_state == DEFAULT_STATE)
      {
        current_state=SOL_STATE;
      }
      break;
    case PVT_STATE:
      if (current_state <= SOL_STATE)
      {
        current_state=PVT_STATE;

      }
      break;
    case MP_STATE:
      if (current_state <= PVT_STATE)
      {
        current_state=MP_STATE;
        printf("MP_STATE\n");
      }
      break;
    case EOL_STATE:
      if (current_state == MP_STATE)
      {
        current_state = EOL_STATE;
        printf("reset\n");
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
        if(buff_counter==BUFFSIZE)
        {
          // change_state_send(send_state);
          change_state_send(send_state);
          if(send_state == EOL_STATE)
          {
            reset_values();
          }
        }
        if(gpio_counter >2)
        {
          gpio_log_send();
          gpio_counter=0;
          memset(&gpio_buff, 0, sizeof(struct gpio_log)*BUFFSIZE);
        }
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
    }else if(ev == sensors_event){
      if(data == &sensor_num1) {
        printf("PC6\n"); //upper inner
        // sensor_upper_time = rtimer_arch_now();
        sensor_upper_time = get_timesynch_time();

        gpio_buff[gpio_counter].gpio=1;
        check_photoelectric_sensors();
        gpio_buff[gpio_counter].state=send_state;
        gpio_counter++;

      }else if(data == &sensor_num2) {
        printf("Pc7\n"); //downer outer
        // sensor_downer_time = rtimer_arch_now();
        sensor_downer_time = get_timesynch_time();

        gpio_buff[gpio_counter].gpio=2;
        check_photoelectric_sensors();
        gpio_buff[gpio_counter].state=send_state;
        gpio_counter++;
      }
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
