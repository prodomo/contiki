#include "collect-view.h"
#include "cc2538-temp-sensor.h"
#include "dev/ain0-sensor.h"

enum {
  SENSOR1,
  SENSOR2,
  SENSOR3,
  SENSOR4,
};

/*---------------------------------------------------------------------------*/
void
collect_view_arch_read_sensors(struct collect_view_data_msg *msg)
{
	int temp_value;
	int ain0_value, ain1_value;
	int battery_value;

	temp_value=cc2538_temp_sensor.value(1);
	ain0_value=ain0_sensor.value(0);
	ain1_value=ain0_sensor.value(1);
	battery_value=0;
  	msg->sensors[SENSOR1] = ain0_value;
  	msg->sensors[SENSOR2] = ain1_value;
  	msg->sensors[SENSOR3] = temp_value;
  	msg->sensors[SENSOR4] = battery_value;
}
/*---------------------------------------------------------------------------*/
