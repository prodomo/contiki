#include "contiki.h"
#include "dev/nvic.h"
#include "dev/ioc.h"
#include "dev/gpio.h"
#include "dev/sensor-gpio2.h"
#include "sys/timer.h"

#include <stdint.h>
#include <string.h>

#define SENSOR_NUM2_PORT_BASE    GPIO_PORT_TO_BASE(SENSOR_NUM2_PORT)
#define SENSOR_NUM2_PIN_MASK     GPIO_PIN_MASK(SENSOR_NUM2_PIN)

/*---------------------------------------------------------------------------*/
static struct timer debouncetimer;
/*---------------------------------------------------------------------------*/
/**
 * \brief Common initialiser for all buttons
 * \param port_base GPIO port's register offset
 * \param pin_mask Pin mask corresponding to the button's pin
 */

 static void
 config(uint32_t port_base, uint32_t pin_mask)
 {
   /* Software controlled */
   GPIO_SOFTWARE_CONTROL(port_base, pin_mask);

   /* Set pin to input */
   GPIO_SET_INPUT(port_base, pin_mask);

   /* Enable edge detection */
   GPIO_DETECT_EDGE(port_base, pin_mask);
   // GPIO_DETECT_LEVEL(port_base, pin_mask);
   // GPIO_TRIGGER_BOTH_EDGES(port_base, pin_mask);

   /* Single edge */
   GPIO_TRIGGER_SINGLE_EDGE(port_base, pin_mask);

   /* Trigger interrupt on Falling edge */
   GPIO_DETECT_RISING(port_base, pin_mask);
   // GPIO_DETECT_FALLING(port_base, pin_mask);

   GPIO_ENABLE_INTERRUPT(port_base, pin_mask);
 }
 /*---------------------------------------------------------------------------*/
 /**
  * \brief Callback registered with the GPIO module. Gets fired with a button
  * port/pin generates an interrupt
  * \param port The port number that generated the interrupt
  * \param pin The pin number that generated the interrupt. This is the pin
  * absolute number (i.e. 0, 1, ..., 7), not a mask
  */
 static void
 sensor_callback(uint8_t port, uint8_t pin)
 {
  if(!timer_expired(&debouncetimer)) {
    return;
  }

  timer_set(&debouncetimer, CLOCK_SECOND / 8);

  if((port == SENSOR_NUM2_PORT) && (pin == SENSOR_NUM2_PIN)) {
    // printf("sensor_num2 change\n");
    sensors_changed(&sensor_num2);
  } 
}
/*---------------------------------------------------------------------------*/
static int
config_sensor_num2(int type, int value)
{
  // printf("config_sensor_num2\n");
  config(SENSOR_NUM2_PORT_BASE, SENSOR_NUM2_PIN_MASK);

  ioc_set_over(SENSOR_NUM2_PORT, SENSOR_NUM2_PIN, IOC_OVERRIDE_PUE);

  NVIC_EnableIRQ(SENSOR_NUM2_VECTOR);

  gpio_register_callback(sensor_callback, SENSOR_NUM2_PORT, SENSOR_NUM2_PIN);
  return 1;
}
/*---------------------------------------------------------------------------*/

void
sensor_gpio2_init()
{
  timer_set(&debouncetimer, 0);
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(sensor_num2, SENSOR_GPIO2, NULL, config_sensor_num2, NULL);
