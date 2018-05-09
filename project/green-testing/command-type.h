#ifndef COMMAND_TYPE_H_
#define COMMAND_TYPE_H_

#define STARTWORD "SW"
#define ENDWORD  "EW"

//BroadCast
#define BROADCAST "FFFF"
//Command Type
#define CMD_TYPE_DATA 0x00
#define CMD_TYPE_CONF 0x01
#define CMD_TYPE_SET 0x02
#define CMD_TYPE_ACK 0x03

//Sensor Tittle 
#define SNR_TLE_DEFAULT 0x00
#define SNR_TLE_TEMPERATURE 0x01
#define SNR_TLE_ELE_CURRENT 0x02
#define SNR_TLE_ROTAT_SPEED 0x03

//Setting Type
// #define SET_TYPE_ASK 0x00
#define SET_TYPE_RATE 0x01
#define SET_TYPE_THRESHOLD 0x02


struct setting_msg {
	uint8_t setting_type;
	uint8_t sensor_tittle;
	uint8_t value;
};

#endif