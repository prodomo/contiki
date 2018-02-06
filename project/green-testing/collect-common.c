/*
 * Copyright (c) 2010, Swedish Institute of Computer Science.
 * All rights reserved.
 *
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
 */

/**
 * \file
 *         Example of how the collect primitive works.
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "contiki.h"
#include "lib/random.h"
#include "net/netstack.h"
#include "dev/serial-line.h"
#include "dev/leds.h"
// #include "net/rime/timesynch.h"
#include "net/mac/tsch/tsch.h"
#include "net/mac/tsch/tsch-queue.h"
#include "net/mac/tsch/tsch-private.h"
#include "net/mac/tsch/tsch-log.h"
#include "net/mac/tsch/tsch-packet.h"
#include "net/mac/tsch/tsch-schedule.h"
#include "net/mac/tsch/tsch-slot-operation.h"
#include "collect-common.h"

#include <stdio.h>
#include <string.h>
#include <ctype.h>

static unsigned long time_offset;
static int send_active = 1;
static int send_command = 0;
static int recv_counter =0;
static char* command_data;

#define COMMAND_PERIOD 10

#ifndef PERIOD
// #define PERIOD 20
#define PERIOD 1
#endif
#define RANDWAIT (PERIOD)

/*---------------------------------------------------------------------------*/
PROCESS(collect_common_process, "collect common process");
AUTOSTART_PROCESSES(&collect_common_process);
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
collect_common_set_send_active(int active)
{
  send_active = active;
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
  printf(" %u %u %u %u",
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
PROCESS_THREAD(collect_common_process, ev, data)
{
  static struct etimer period_timer, wait_timer, command_timer;
  PROCESS_BEGIN();

  collect_common_net_init();

  /* Send a packet every 60-62 seconds. */
  etimer_set(&period_timer, CLOCK_SECOND * PERIOD);
  etimer_set(&command_timer, CLOCK_SECOND * COMMAND_PERIOD);
  while(1) {
    PROCESS_WAIT_EVENT();
    if(ev == serial_line_event_message) {
      char *line;
      line = (char *)data;
      printf("--------------rev command------------:%s\n", line);
      if(strncmp(line, "send", 4)==0)
      {
        send_command=1;
        recv_counter++;
        command_data = malloc(strlen(line) + 1);
        strcpy(command_data, line);
        printf("recv_counter: %d\n", recv_counter);
        printf("command_data: %s\n", command_data);
        printf("-----------recv send command-----------\n");

      // if(strncmp(line, "collect", 7) == 0 ||
      //    strncmp(line, "gw", 2) == 0) {
      //   collect_common_set_sink();
      // } else if(strncmp(line, "net", 3) == 0) {
      //   collect_common_net_print();
      // } else if(strncmp(line, "time ", 5) == 0) {
      //   unsigned long tmp;
      //   line += 6;
      //   while(*line == ' ') {
      //     line++;
      //   }
      //   tmp = strtolong(line);
      //   time_offset = clock_seconds() - tmp;
      //   printf("Time offset set to %lu\n", time_offset);
      // } else if(strncmp(line, "mac ", 4) == 0) {
      //   line +=4;
      //   while(*line == ' ') {
      //     line++;
      //   }
      //   if(*line == '0') {
      //     NETSTACK_RDC.off(1);
      //     printf("mac: turned MAC off (keeping radio on): %s\n",
      //            NETSTACK_RDC.name);
      //   } else {
      //     NETSTACK_RDC.on();
      //     printf("mac: turned MAC on: %s\n", NETSTACK_RDC.name);
      //   }

      // } else if(strncmp(line, "~K", 2) == 0 ||
      //           strncmp(line, "killall", 7) == 0) {
      //   /* Ignore stop commands */
      } else {
        printf("unhandled command: %s\n", line);
      }
    }
    if(ev == PROCESS_EVENT_TIMER) {
      if(data == &period_timer) {
        etimer_reset(&period_timer);
        etimer_set(&wait_timer, random_rand() % (CLOCK_SECOND * RANDWAIT));
      } else if(data == &wait_timer) {
        if(send_active) {
          /* Time to send the data */
          collect_common_send();
        }
      }else if(data == &command_timer){
        printf("command_timer timeup");
        etimer_reset(&command_timer);
        if(send_command==1){
            printf("send_command = 1 \n");
            collect_special_send(command_data);
            free(command_data);
            send_command = 0;
          }
        }
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
