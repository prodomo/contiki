/**
 * \file
 *      Temperature resource
 * \author
 *      White
 */

#include <string.h>
#include "rest-engine.h"
#include "er-coap.h"
#include "sys/clock.h"
#include "lib/sensors.h"
#include "dev/sht21.h"
#include "dev/max44009.h"
#include "dev/adxl346.h"

#define DEBUG 1
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

PERIODIC_RESOURCE(res_temperature,
                  "title=\"Periodic collect\";obs",
                  res_get_handler,
                  res_post_handler,
                  NULL,
                  NULL,
                  2 * CLOCK_SECOND,
                  res_periodic_handler);

/*
 * Use local resource state that is accessed by res_get_handler() and altered by res_periodic_handler() or PUT or POST.
 */
static int32_t event_counter = 0;

/* inter-packet time we generate a packet to send to observer */
static int8_t event_threshold = 5;

/* record last change event threshold's event_counter */
static int32_t event_threshold_last_change = 0;

/* Record the packet have been generated. (Server perspective) */
static int32_t packet_counter = 0;

static void
res_get_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
  /*
   * For minimal complexity, request query and options should be ignored for GET on observable resources.
   * Otherwise the requests must be stored with the observer list and passed by REST.notify_subscribers().
   * This would be a TODO in the corresponding files in contiki/apps/erbium/!
   */

  static int8_t sht21_present=0, max44009_present=0, adxl346_present=0; 
  static int16_t temperature, humidity, light, accelx, accely, accelz;

  if(sht21.status(SENSORS_READY)==1) {
        temperature = sht21.value(SHT21_READ_TEMP);
        //PRINTF("Temperature: %u.%uC\n", temperature / 100, temperature % 100);
        humidity = sht21.value(SHT21_READ_RHUM);
        //PRINTF("Rel. humidity: %u.%u%%\n", humidity / 100, humidity % 100);
        sht21_present = 1;
  }

  if(max44009.status(SENSORS_READY)==1) {

        light = max44009.value(MAX44009_READ_LIGHT);
        printf("Light: %u.%ulux\n", light / 100, light % 100);
        max44009_present = 1;
  }

  if(adxl346.status(SENSORS_READY)==1) {
        //leds_on(LEDS_YELLOW);
        accelx = adxl346.value(ADXL346_READ_X_mG);
        //printf("X Acceleration: %d.%u G\n", accel / 1000, accel % 1000);
        accely = adxl346.value(ADXL346_READ_Y_mG);
        //printf("Y Acceleration: %d.%u G\n", accel / 1000, accel % 1000);
        accelz = adxl346.value(ADXL346_READ_Z_mG);
        //printf("Z Acceleration: %d.%u G\n", accel / 1000, accel % 1000);
        //leds_off(LEDS_YELLOW);
        adxl346_present = 1;
  }


  PRINTF("I am collect res_get hanlder!\n");
  REST.set_header_content_type(response, REST.type.TEXT_PLAIN);
  REST.set_header_max_age(response, res_temperature.periodic->period / CLOCK_SECOND);

 if (sht21_present && max44009_present && adxl346_present){

    REST.set_response_payload(response, buffer, snprintf((char *)buffer, preferred_size, "packet_time : %d\n temperature : %u\n humidity : %u\n Light: %u\n ax : %u\n ay : %u\n az : %u\n", event_threshold,temperature,humidity,light,accelx,accely,accelz));

 }else if (sht21_present && max44009_present){
    REST.set_response_payload(response, buffer, snprintf((char *)buffer, preferred_size, "packet_time : %d\n temperature : %u\n humidity : %u\n Light: %u\n And adxl346 doesn't work", event_threshold,temperature,humidity,light));
 }else if (sht21_present && adxl346_present){
    REST.set_response_payload(response, buffer, snprintf((char *)buffer, preferred_size, "packet_time : %d\n temperature : %u\n humidity : %u\n ax : %u\n ay : %u\n az : %u\n And max44009 doesn't work", event_threshold,temperature,humidity,accelx,accely,accelz));
 }else if (max44009_present && adxl346_present){
    REST.set_response_payload(response, buffer, snprintf((char *)buffer, preferred_size, "packet_time : %d\n Light: %u\n ax : %u\n ay : %u\n az : %u\n And sht21 doesn't work", event_threshold,light,accelx,accely,accelz));
 }else if (sht21_present){
    REST.set_response_payload(response, buffer, snprintf((char *)buffer, preferred_size, "packet_time : %d\n temperature : %u\n humidity : %u\n  And max44009,adxl346 don't work", event_threshold,temperature,humidity));
 }else if (max44009_present){
    REST.set_response_payload(response, buffer, snprintf((char *)buffer, preferred_size, "packet_time : %d\n Light: %u\n And sht21,adxl346 don't work", event_threshold,light,accelx,accely,accelz));
 }else if (adxl346_present){
     REST.set_response_payload(response, buffer, snprintf((char *)buffer, preferred_size, "packet_time : %d\n ax : %u\n ay : %u\n az : %u\n And sht21,max44009 don't work", event_threshold,accelx,accely,accelz));
 }
 else{
    REST.set_response_payload(response, buffer, snprintf((char *)buffer, preferred_size, "packet_time : %d\n ax : %u\n ay : %u\n az : %u\n And sht21,max44009,adxl346 don't work", event_threshold));
 }

  
  //REST.set_response_payload(response, buffer, snprintf((char *)buffer, preferred_size, "packet_time : %d\n ", event_threshold));

  /* The REST.subscription_handler() will be called for observable resources by the REST framework. */
}


/* Used for update the threshold */
static void
res_post_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
  const char *threshold_c = NULL;
  int threshold = -1;
  if(REST.get_query_variable(request, "threshold", &threshold_c)) {
    threshold = (int8_t)atoi(threshold_c);
  }

  if(threshold < 1) {
    /* Threashold is too smaill ignore it! */
    REST.set_response_status(response, REST.status.BAD_REQUEST);
  } else {
    /* Update to new threshold */
    event_threshold = threshold;
    event_threshold_last_change = event_counter;
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
  
#if CONTIKI_TARGET_CC2538DK
  static clock_time_t clock_g;
  rtimer_clock_t rtimer_now;
  // clock_time_t timer_start = periodic_res_collect.periodic_timer.timer.start;

#endif

  /* Will notify subscribers when inter-packet time is match */
  if(event_counter % event_threshold == 0) {
    ++packet_counter;
    PRINTF("Generate a new packet!\n");
    
    

    /* Notify the registered observers which will trigger the res_get_handler to create the response. */
    REST.notify_subscribers(&res_temperature);
  }
}