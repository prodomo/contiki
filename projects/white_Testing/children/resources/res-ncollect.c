/**
 * \file
 *      Collect resource
 * \author
 *      Green
 */

#include <string.h>
#include "rest-engine.h"
#include "er-coap.h"
#include "sys/clock.h"

#include "core/net/rpl/rpl.h"
#include "core/net/link-stats.h"

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#define PRINT6ADDR(addr) PRINTF("[%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x]", ((uint8_t *)addr)[0], ((uint8_t *)addr)[1], ((uint8_t *)addr)[2], ((uint8_t *)addr)[3], ((uint8_t *)addr)[4], ((uint8_t *)addr)[5], ((uint8_t *)addr)[6], ((uint8_t *)addr)[7], ((uint8_t *)addr)[8], ((uint8_t *)addr)[9], ((uint8_t *)addr)[10], ((uint8_t *)addr)[11], ((uint8_t *)addr)[12], ((uint8_t *)addr)[13], ((uint8_t *)addr)[14], ((uint8_t *)addr)[15])
#define PRINTLLADDR(lladdr) PRINTF("[%02x:%02x:%02x:%02x:%02x:%02x]", (lladdr)->addr[0], (lladdr)->addr[1], (lladdr)->addr[2], (lladdr)->addr[3], (lladdr)->addr[4], (lladdr)->addr[5])
#else
#define PRINTF(...)
#define PRINT6ADDR(addr)
#define PRINTLLADDR(addr)
#endif


static void res_get_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset);
static void res_post_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset);
static void res_periodic_handler(void);

PERIODIC_RESOURCE(res_ncollect,
                  "title=\"Periodic collect\";obs",
                  res_get_handler,
                  res_post_handler,
                  NULL,
                  NULL,
                  1 * CLOCK_SECOND,
                  res_periodic_handler);

/*
 * Use local resource state that is accessed by res_get_handler() and altered by res_periodic_handler() or PUT or POST.
 */
static uint16_t event_counter = 0;

/* inter-packet time we generate a packet to send to observer */
static uint16_t event_threshold = 20;

/* record last change event threshold's event_counter */
static uint16_t event_threshold_last_change = 0;

/* Record the packet have been generated. (Server perspective) */
static uint16_t packet_counter = 0;

static uint16_t packet_priority = 0;

static uint16_t battery_threshold = 3;

static uint16_t sensor_threshold = 50;

static void
res_get_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
  /*
   * For minimal complexity, request query and options should be ignored for GET on observable resources.
   * Otherwise the requests must be stored with the observer list and passed by REST.notify_subscribers().
   * This would be a TODO in the corresponding files in contiki/apps/erbium/!
   */

  struct 
  {
    uint16_t packet_counter;
    uint16_t rank;
    unsigned char parent_address[2];
    uint16_t parent_etx;
    uint16_t num_neighbors;
    int16_t rssi;
    uint16_t battery;
    uint16_t battery_threshold;
    uint16_t int_tempature_value;
    uint16_t ext_tempature_value;
    uint16_t sensor_threshold;
    uint16_t period;
    uint16_t priority;
  } message;

  memset(&message, 0, sizeof(message));
  
  message.packet_counter = packet_counter;
  message.period = event_threshold;
  message.priority = packet_priority;
  message.battery_threshold = battery_threshold;
  message.sensor_threshold = sensor_threshold;

  message.battery = 5;
  message.int_tempature_value = 30;
  message.ext_tempature_value = 30;

  uint8_t packet_length = 0;
  rpl_dag_t *dag;
  rpl_parent_t *preferred_parent;
  linkaddr_t parent;
  linkaddr_copy(&parent, &linkaddr_null);
  struct link_stats *parent_link_stats;


  PRINTF("I am N_collect res_get hanlder!\n");
  REST.set_header_content_type(response, REST.type.APPLICATION_OCTET_STREAM);
  REST.set_header_max_age(response, res_ncollect.periodic->period / CLOCK_SECOND);

  

  // packet_counter
  // memcpy(buffer,&packet_counter, sizeof(packet_counter));
  // packet_counter += sizeof(packet_counter);

  
  dag = rpl_get_any_dag();
  if(dag != NULL) {
    preferred_parent = dag->preferred_parent;
    if(preferred_parent != NULL) {
      uip_ds6_nbr_t *nbr;
      nbr = uip_ds6_nbr_lookup(rpl_get_parent_ipaddr(preferred_parent));
      if(nbr != NULL) {
        /* Use parts of the IPv6 address as the parent address, in reversed byte order. */
        parent.u8[LINKADDR_SIZE - 1] = nbr->ipaddr.u8[sizeof(uip_ipaddr_t) - 2];
        parent.u8[LINKADDR_SIZE - 2] = nbr->ipaddr.u8[sizeof(uip_ipaddr_t) - 1];
        parent_link_stats = rpl_get_parent_link_stats(preferred_parent);
        message.parent_etx = parent_link_stats->etx;
        message.rssi = parent_link_stats->rssi;
      }
    }
    message.rank = dag->rank;
    message.num_neighbors = uip_ds6_nbr_num();
  } else {
    message.rank = 0;
    message.num_neighbors = 0;
  }
  message.parent_address[0] = parent.u8[LINKADDR_SIZE - 1];
  message.parent_address[1] = parent.u8[LINKADDR_SIZE - 2];

  memcpy(buffer, &message, sizeof(message));

  printf("payload size: %d\n", sizeof(message));


  coap_set_uip_traffic_class(packet_priority);
  REST.set_response_payload(response, buffer, sizeof(message));
  /* The REST.subscription_handler() will be called for observable resources by the REST framework. */
}


/* Used for update the threshold */
static void
res_post_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
  const char *threshold_c = NULL;
  const char *priority_c = NULL;
  int threshold = -1;
  int priority = -1;

  if(REST.get_query_variable(request, "thd", &threshold_c)) {
    threshold = (uint8_t)atoi(threshold_c);
  }

  if(REST.get_query_variable(request, "pp", &priority_c)) {
    priority = (uint8_t)atoi(priority_c);
  }

  if(threshold < 1 && (priority<0||priority>2)) {
    /* Threashold is too smaill ignore it! */
    REST.set_response_status(response, REST.status.BAD_REQUEST);
  } else {
    if(threshold>=1){
      /* Update to new threshold */
      event_threshold = threshold;
      event_threshold_last_change = event_counter;
    }
    if(priority>=0 && priority<= 2)
    {
      packet_priority = priority;
    }
  }
}
/*
 * Additionally, a handler function named [resource name]_handler must be implemented for each PERIODIC_RESOURCE.
 * It will be called by the REST manager process with the defined period.
 */
static void
res_periodic_handler()
{
  /* This periodic handler will be called every second */
  ++event_counter;

  /* Will notify subscribers when inter-packet time is match */
  if(event_counter % event_threshold == 0) {
    ++packet_counter;
    PRINTF("Generate a new packet! , %08x. \n",tsch_current_asn.ls4b);
        
    /* Notify the registered observers which will trigger the res_get_handler to create the response. */
    REST.notify_subscribers(&res_ncollect);
  }
}
