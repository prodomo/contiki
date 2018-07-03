#ifndef SENSOR_GPIO_H_
#define SENSOR_GPIO_H_

#include "lib/sensors.h"
#include "dev/gpio.h"

#define SENSOR_GPIO "SENGPIO"

#define sensor_gpio sensor_num1
extern const struct sensors_sensor sensor_num1;
extern const struct sensors_sensor sensor_num2;

/*---------------------------------------------------------------------------*/
#endif /* SENSOR_GPIO_H_ */

/** \brief Common initialiser for all SmartRF Buttons */
