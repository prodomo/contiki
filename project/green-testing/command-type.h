#ifndef COMMAND_TYPE_H_
#define COMMAND_TYPE_H_

#define STARTWORD "SW"
#define ENDWORD  "EW"

//BroadCast
#define BROADCAST "FFFF"
//Command Type
#define CMD_TYPE_DATA 0x00
#define CMD_TYPE_CONF 0x01
#define CMD_TYPE_SET  0x02
#define CMD_TYPE_ACK  0x03

//Sensor Tittle 
#define SNR_TLE_DEFAULT 	0x00
#define SNR_TLE_TEMPERATURE 0x01
#define SNR_TLE_ELE_CURRENT 0x02
#define SNR_TLE_ROTAT_SPEED 0x03
#define SNR_TLE_DISTANCE    0x04

#define SNR_TLE_MODBUS_ADDR 0x11
#define SNR_TLE_MODBUS_REG  0x12
#define SNR_TLE_MODBUS_FREQ 0x13
#define SNR_TLE_MODBUS_TYPE 0x14

//Setting Type
// #define SET_TYPE_ASK 0x00
#define SET_TYPE_RATE 		0x01
#define SET_TYPE_THRESHOLD  0x02
#define SET_TYPE_STATE 		0x03
#define SET_TYPE_MODBUS		0x04

//GENERATATION STATE
#define DEFAULT_STATE 0x00
#define SOL_STATE 	  0x01  //Start of Life
#define PVT_STATE     0x02  //Production Validation Test 
#define MP_STATE      0x03  //Mass Production
#define EOMP_STATE    0x04  //End of Life
#define EOL_STATE     0x05  //End of Life

#define START_CLOSE   0x05
#define CLOSE         0x06
#define START_OPEN	  0x07
#define OPEN          0x08


struct setting_msg {
	uint16_t setting_type;
	uint16_t sensor_tittle;
	uint16_t value;
};

struct  gpio_log
{
	uint16_t	gpio;
	uint16_t	state;
	
};

#endif