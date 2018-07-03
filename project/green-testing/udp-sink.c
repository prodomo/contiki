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
#include "contiki-lib.h"
#include "contiki-net.h"
#include "net/ip/uip.h"
#include "net/rpl/rpl.h"
#include "net/linkaddr.h"
#include "net/rpl/rpl-private.h"

#include "net/netstack.h"
// #include "dev/button-sensor.h"
#include "dev/serial-line.h"
#if CONTIKI_TARGET_Z1
#include "dev/uart0.h"
#else
#include "dev/uart1.h"
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <string.h>
// #include "collect-common.h"
#include "collect-view.h"
#include "command-type.h"
#include "net/mac/tsch/tsch.h"
#include "net/mac/tsch/tsch-queue.h"
#include "net/mac/tsch/tsch-private.h"
#include "net/mac/tsch/tsch-log.h"
#include "net/mac/tsch/tsch-packet.h"
#include "net/mac/tsch/tsch-schedule.h"
#include "net/mac/tsch/tsch-slot-operation.h"


#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"

#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

#define UDP_CLIENT_PORT 8775
#define UDP_SERVER_PORT 5688

static struct uip_udp_conn *server_conn;
static uip_ipaddr_t client_ipaddr;

static unsigned long time_offset;
static int send_command = 0;
static int recv_counter =0;
static char* command_data;

#define COMMAND_PERIOD 2

static uint16_t command_id=0;

PROCESS(udp_server_process, "UDP server process");
AUTOSTART_PROCESSES(&udp_server_process);
/*---------------------------------------------------------------------------*/
void
collect_common_net_print(void)
{
  printf("I am sink!\n");
}
/*---------------------------------------------------------------------------*/
uint8_t
ascii_to_uint(char c)
{
  uint8_t result;
  uint8_t temp;
  temp = (uint8_t)c;
  if(temp<58)
    result = temp-'0';
  else if(temp<71 && temp>64)
    result = temp-'7';
  else if(temp<103  && temp>96)
    result = temp-'W';
  else
    result=temp;

  return result;
}

/*---------------------------------------------------------------------------*/
void
collect_setting_send(char* data)
{
  rpl_dag_t *dag;
  char* split;
  char temp[20][20];
  int count=0;
  uint8_t  dst_u8[2];
  uip_ipaddr_t addr;

  /*----------parsing command--------- */
  /*sw commandType mac commandid sensornum ...*/
  split = strtok (data," ,.-\\");
  while (split != NULL)
  {
    strcpy(temp[count], split);
    count++;
    split = strtok (NULL, " ,.-\\");
  }

  int sensor_num = atoi(temp[4]);
  // printf("senor_num %d\n", sensor_num);

  struct msg
  {
    uint16_t commandId;
    uint16_t commandType;
    uint16_t sensorNum;
    struct setting_msg setmsg[sensor_num];
  };
  struct msg msg;
  memset(&msg, 0, sizeof(msg));

  msg.commandType = CMD_TYPE_SET;
  msg.commandId = (uint16_t)atoi(temp[3]);
  msg.sensorNum = sensor_num;
  printf("msg.commandType %d\n", msg.commandType);
  printf("msg.commandId %d\n", msg.commandId);
  printf("sensor_num%d\n", sensor_num);

  for(int i=0; i<sensor_num; i++)
  {
    msg.setmsg[i].setting_type = atoi(temp[5+i*3]);
    msg.setmsg[i].sensor_tittle = atoi(temp[6+i*3]);
    msg.setmsg[i].value = atoi(temp[7+i*3]);
    printf("setting_type %d\n", msg.setmsg[i].setting_type);
    printf("sensor_tittle %d\n", msg.setmsg[i].sensor_tittle);
    printf("value %d\n", msg.setmsg[i].value);
  }

  if(server_conn == NULL) {
    /* Not setup yet */
    return;
  }

  printf("sizeof(msg):%d\n", sizeof(msg));
  // send_packet(&msg, temp[2], sizeof(msg));
  leds_toggle(LEDS_RED);

  uip_ipaddr_copy(&client_ipaddr, &UIP_IP_BUF->srcipaddr);

  if(strncmp(temp[2], BROADCAST, 4)==0)
  {
    //broadcast command
    // printf("broadcast\n");
    uip_create_linklocal_rplnodes_mcast(&addr);
    printf("sizeof(msg) %d\n", sizeof(msg));
    uip_udp_packet_sendto(server_conn, &msg, sizeof(msg),
                        &addr, UIP_HTONS(UDP_CLIENT_PORT));
  }
  else
  {
    //unicast command 
    /* assume temp[1] is mac addr */
    /* ascii -> uint8 */
    dst_u8[0] =ascii_to_uint(temp[2][0])<<4;
    dst_u8[0] += ascii_to_uint(temp[2][1]);

    dst_u8[1] = ascii_to_uint(temp[2][2])<<4;
    dst_u8[1] += ascii_to_uint(temp[2][3]);
      
    printf("%02x%02x\n", dst_u8[0], dst_u8[1]);

     
    // /* destnation ipv6 address */
    client_ipaddr.u8[0]=0xfe;
    client_ipaddr.u8[1]=0x80;
    client_ipaddr.u8[8]=0x02;
    client_ipaddr.u8[9]=0x12;
    client_ipaddr.u8[10]=0x4b;
    client_ipaddr.u8[11]=0x00;
    client_ipaddr.u8[12]=0x06;
    // client_ipaddr.u8[13]=0x0d; //openMote
    client_ipaddr.u8[13]=0x15; //ITRI_Mote
    client_ipaddr.u8[14]=dst_u8[0];
    client_ipaddr.u8[15]=dst_u8[1];
    // printf("\n-----------------------\n");
    // printf("client_ipaddr2:");
    // PRINT6ADDR(&client_ipaddr);
    // printf("\n-----------------------\n");
    printf("sizeof(msg) %d\n", sizeof(msg));
    uip_udp_packet_sendto(server_conn, &msg, sizeof(msg),
                          &client_ipaddr, UIP_HTONS(UDP_CLIENT_PORT));

  }

}
/*---------------------------------------------------------------------------*/
void
collect_ask_send(char* mac, char* commandId)
{
  // printf("collect_ask_send\n");

  uint8_t  dst_u8[2];
  uip_ipaddr_t addr;
  struct msg{
    uint16_t commandId;
    uint16_t commandType;
  };

  struct msg msg;
  memset(&msg, 0, sizeof(msg));

  msg.commandType = CMD_TYPE_CONF;
  msg.commandId = (uint16_t)atoi(commandId);
  printf("msg.commandType %d\n", msg.commandType);
  printf("msg.commandId %d\n", msg.commandId);

  if(server_conn == NULL) {
    /* Not setup yet */
    return;
  }

  // send_packet(&msg, mac, sizeof(msg));

  uip_ipaddr_copy(&client_ipaddr, &UIP_IP_BUF->srcipaddr);

  if(strncmp(mac, BROADCAST, 4)==0)
  {
    //broadcast command
    // printf("broadcast\n");
    uip_create_linklocal_rplnodes_mcast(&addr);
    printf("sizeof(msg) %d\n", sizeof(msg));
    uip_udp_packet_sendto(server_conn, &msg, sizeof(msg),
                        &addr, UIP_HTONS(UDP_CLIENT_PORT));
  }
  else
  {
    //unicast command 
    /* assume temp[1] is mac addr */
    /* ascii -> uint8 */
    dst_u8[0] =ascii_to_uint(mac[0])<<4;
    dst_u8[0] += ascii_to_uint(mac[1]);

    dst_u8[1] = ascii_to_uint(mac[2])<<4;
    dst_u8[1] += ascii_to_uint(mac[3]);
      
    printf("%02x%02x\n", dst_u8[0], dst_u8[1]);

     
    // /* destnation ipv6 address */
    client_ipaddr.u8[0]=0xfe;
    client_ipaddr.u8[1]=0x80;
    client_ipaddr.u8[8]=0x02;
    client_ipaddr.u8[9]=0x12;
    client_ipaddr.u8[10]=0x4b;
    client_ipaddr.u8[11]=0x00;
    client_ipaddr.u8[12]=0x06;
    // client_ipaddr.u8[13]=0x0d; //openMote
    client_ipaddr.u8[13]=0x15; //ITRI_Mote
    client_ipaddr.u8[14]=dst_u8[0];
    client_ipaddr.u8[15]=dst_u8[1];
    // printf("\n-----------------------\n");
    // printf("client_ipaddr2:");
    // PRINT6ADDR(&client_ipaddr);
    // printf("\n-----------------------\n");
    printf("sizeof(msg) %d\n", sizeof(msg));
    uip_udp_packet_sendto(server_conn, &msg, sizeof(msg),
                          &client_ipaddr, UIP_HTONS(UDP_CLIENT_PORT));

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

  PRINTF("I am sink!\n");
}
/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{
  uint8_t *appdata;
  linkaddr_t sender;
  uint8_t seqno;
  uint8_t hops;
  uint16_t commandId;
  uint16_t data;

  if(uip_newdata()) {
    appdata = (uint8_t *)uip_appdata;
    sender.u8[0] = UIP_IP_BUF->srcipaddr.u8[15];
    sender.u8[1] = UIP_IP_BUF->srcipaddr.u8[14];
    seqno = *appdata;
    hops = uip_ds6_if.cur_hop_limit - UIP_IP_BUF->ttl + 1;
    printf("receive packet length %d\r\n", uip_datalen());
    // if(uip_datalen()>40)
    // {
    //   collect_common_recv(&sender, seqno, hops,
    //                     appdata + 2, uip_datalen() - 2);
    // }
    // else
    // {
      printf("%u ", uip_datalen());
      printf("%04x ", sender.u8[0] + (sender.u8[1] << 8));
      memcpy(&commandId, appdata, sizeof(uint16_t));
      appdata+=sizeof(uint16_t);
      printf("%u ",commandId);
      for(int i=0; i<(uip_datalen()-2)/2; i++)
      {
        memcpy(&data , appdata, sizeof(uint16_t));
        appdata+=sizeof(uint16_t);
        printf("%u ",data);
      }
      printf("\n");
    // }
  }
}
/*---------------------------------------------------------------------------*/
static void
print_local_addresses(void)
{
  int i;
  uint8_t state;

  PRINTF("Server IPv6 addresses: ");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(state == ADDR_TENTATIVE || state == ADDR_PREFERRED) {
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
void
collect_command_parse(char* data)
{
  char* split;
  char temp[20][20];
  int count=0;
  
  char* copy = malloc(strlen(data) + 1);
  strcpy(copy, data);

  split = strtok (data," ,.-\\");
  while (split != NULL)
  {
    // printf ("%s %d\n",split, strlen(split));
    strcpy(temp[count], split);
    count++;
    split = strtok (NULL, " ,.-\\");
  }

  switch(atoi(temp[1]))
  {
    case CMD_TYPE_CONF:
    {
      printf("collect_ask_send\n");
      collect_ask_send(temp[2], temp[3]);
      break;
    }
    case CMD_TYPE_SET:
    {
      printf("collect_setting_send\n");
      printf("copy %s\n", copy);
      collect_setting_send(copy);
      free(copy);
      break;
    }
    default:
      break;
  }
}
/*---------------------------------------------------------------------------*/
void
collect_common_recv(const linkaddr_t *originator, uint8_t seqno, uint8_t hops,
                    uint8_t *payload, uint16_t payload_len)
{
  unsigned long time;
  struct tsch_asn_t asn;
  uint16_t data;
  int i;

  printf("%u", 8 + payload_len / 2);
  /* Timestamp. Ignore time synch for now. */
  time=get_time();
  printf(" %lu %lu 0", ((time >> 16) & 0xffff), time & 0xffff);
  /* Ignore latency for now */
  asn = get_timesynch_time();
  printf(" %04x %u %u %u",
         originator->u8[0] + (originator->u8[1] << 8), seqno, hops, asn.ls4b);
  for(i = 0; i < payload_len / 2; i++) {
    memcpy(&data, payload, sizeof(data));
    payload += sizeof(data);
    printf(" %u", data);
  }
  printf("\n");
  leds_blink();
}
/*---------------------------------------------------------------------------*/
void
collect_ack_recv(const linkaddr_t *originator, uint8_t *payload)
{
  uint8_t data;
  int i;
  struct 
  {
    uint16_t command_type;
    uint16_t data_length;
    uint16_t command_id;
    uint16_t is_received;
  }ack;
  printf("%u ",originator->u8[0] + (originator->u8[1] << 8)); //nodeID
  memcpy(&ack, payload, sizeof(ack));
  printf("%u %u %u %u\n", ack.command_type, ack.data_length, ack.command_id, ack.is_received);
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_server_process, ev, data)
{
  uip_ipaddr_t ipaddr;
  struct uip_ds6_addr *root_if;
  static struct etimer command_timer;

  PROCESS_BEGIN();
  collect_common_net_init();

  PROCESS_PAUSE();

  // SENSORS_ACTIVATE(button_sensor);

  PRINTF("UDP server started\n");

#if UIP_CONF_ROUTER
  uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 1);
  /* uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr); */
  uip_ds6_addr_add(&ipaddr, 0, ADDR_MANUAL);
  root_if = uip_ds6_addr_lookup(&ipaddr);
  if(root_if != NULL) {
    rpl_dag_t *dag;
    dag = rpl_set_root(RPL_DEFAULT_INSTANCE,(uip_ip6addr_t *)&ipaddr);
    uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
    rpl_set_prefix(dag, &ipaddr, 64);
    PRINTF("created a new RPL dag\n");
  } else {
    PRINTF("failed to create a new RPL DAG\n");
  }
#endif /* UIP_CONF_ROUTER */

  print_local_addresses();

  /* The data sink runs with a 100% duty cycle in order to ensure high
     packet reception rates. */
  NETSTACK_MAC.on();
  // NETSTACK_RDC.off(1);

  server_conn = udp_new(NULL, UIP_HTONS(UDP_CLIENT_PORT), NULL);
  udp_bind(server_conn, UIP_HTONS(UDP_SERVER_PORT));

  PRINTF("Created a server connection with remote address ");
  PRINT6ADDR(&server_conn->ripaddr);
  PRINTF(" local/remote port %u/%u\n", UIP_HTONS(server_conn->lport),
         UIP_HTONS(server_conn->rport));

    /* Send a packet every 60-62 seconds. */
  etimer_set(&command_timer, CLOCK_SECOND * COMMAND_PERIOD);

  while(1) {
    PROCESS_YIELD();
    if(ev == tcpip_event) {
      tcpip_handler();
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
    else if(ev == PROCESS_EVENT_TIMER) {
      if(data == &command_timer){
        // printf("command_timer timeup\n");
        etimer_reset(&command_timer);
        if(send_command==1){
            printf("send_command = 1 \n");
            collect_command_parse(command_data); //sink special send
            free(command_data);
            send_command = 0;
          }
        }
      }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
