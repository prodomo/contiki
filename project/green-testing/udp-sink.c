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
#include "dev/button-sensor.h"
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
#include "collect-common.h"
#include "collect-view.h"
#include "command-type.h"

#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"

#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

#define UDP_CLIENT_PORT 8775
#define UDP_SERVER_PORT 5688

static struct uip_udp_conn *server_conn;
static uip_ipaddr_t client_ipaddr;

PROCESS(udp_server_process, "UDP server process");
AUTOSTART_PROCESSES(&udp_server_process,&collect_common_process);
/*---------------------------------------------------------------------------*/
void
collect_common_set_sink(void)
{
}
/*---------------------------------------------------------------------------*/
void
collect_common_net_print(void)
{
  printf("I am sink!\n");
}
/*---------------------------------------------------------------------------*/
void
collect_common_send(void)
{
  /* Server never sends */
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
    uint8_t commandType;
    uint8_t sensorNum;
    struct setting_msg setmsg[sensor_num];
  };
  struct msg msg;
  memset(&msg, 0, sizeof(msg));

  msg.commandType = CMD_TYPE_SET;
  msg.commandId = (uint16_t)atoi(temp[3]);
  msg.sensorNum = sensor_num;
  // printf("msg.commandType %d\n", msg.commandType);
  // printf("msg.commandId %d\n", msg.commandId);

  for(int i=0; i<sensor_num; i++)
  {
    msg.setmsg[i].setting_type = atoi(temp[5+i*3]);
    msg.setmsg[i].sensor_tittle = atoi(temp[6+i*3]);
    msg.setmsg[i].value = atoi(temp[7+i*3]);
    // printf("setting_type %d\n", msg.setmsg[i].setting_type);
    // printf("sensor_tittle %d\n", msg.setmsg[i].sensor_tittle);
    // printf("value %d\n", msg.setmsg[i].value);
  }

  if(server_conn == NULL) {
    /* Not setup yet */
    return;
  }
  uip_ipaddr_copy(&client_ipaddr, &UIP_IP_BUF->srcipaddr);
  
  if(strncmp(temp[2], BROADCAST, 4)==0)
  {
    //broadcast command
    // printf("broadcast\n");
    uip_create_linklocal_rplnodes_mcast(&addr);
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
    uip_udp_packet_sendto(server_conn, &msg, sizeof(msg),
                          &client_ipaddr, UIP_HTONS(UDP_CLIENT_PORT));

  }

  leds_toggle(LEDS_RED);

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
    uint8_t commandType;
  };

  struct msg msg;
  memset(&msg, 0, sizeof(msg));

  msg.commandType = CMD_TYPE_CONF;
  msg.commandId = (uint16_t)atoi(commandId);

  if(server_conn == NULL) {
    /* Not setup yet */
    return;
  }

  uip_ipaddr_copy(&client_ipaddr, &UIP_IP_BUF->srcipaddr);

  if(strncmp(mac, BROADCAST, 4)==0)
  {
    //broadcast command
    // printf("broadcast\n");
    uip_create_linklocal_rplnodes_mcast(&addr);
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
    uip_udp_packet_sendto(server_conn, &msg, sizeof(msg),
                          &client_ipaddr, UIP_HTONS(UDP_CLIENT_PORT));

  }
}
/*---------------------------------------------------------------------------*/
void
collect_ack_send(uint16_t commandId)
{
  /*sink not send ack*/
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
  uint8_t data;

  if(uip_newdata()) {
    appdata = (uint8_t *)uip_appdata;
    sender.u8[0] = UIP_IP_BUF->srcipaddr.u8[15];
    sender.u8[1] = UIP_IP_BUF->srcipaddr.u8[14];
    seqno = *appdata;
    hops = uip_ds6_if.cur_hop_limit - UIP_IP_BUF->ttl + 1;
    // printf("receive packet length %d\n", uip_datalen());
    if(uip_datalen()>40)
    {
      collect_common_recv(&sender, seqno, hops,
                        appdata + 2, uip_datalen() - 2);
    }
    else if(uip_datalen()==4)
    {
      printf("%u ", uip_datalen());
      printf("%04x ", sender.u8[0] + (sender.u8[1] << 8));
      memcpy(&commandId, appdata, sizeof(uint16_t));
      appdata+=sizeof(uint16_t);
      printf("%u ",commandId);
      for(int i=0; i<uip_datalen()-2; i++)
      {
        memcpy(&data , appdata, sizeof(uint8_t));
        appdata+=sizeof(uint8_t);
        printf("%u ",data);
      }
      printf("\n");
    }
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
PROCESS_THREAD(udp_server_process, ev, data)
{
  uip_ipaddr_t ipaddr;
  struct uip_ds6_addr *root_if;

  PROCESS_BEGIN();

  PROCESS_PAUSE();

  SENSORS_ACTIVATE(button_sensor);

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

  while(1) {
    PROCESS_YIELD();
    if(ev == tcpip_event) {
      tcpip_handler();
    } else if (ev == sensors_event && data == &button_sensor) {
      PRINTF("Initiating global repair\n");
      rpl_repair_root(RPL_DEFAULT_INSTANCE);
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
