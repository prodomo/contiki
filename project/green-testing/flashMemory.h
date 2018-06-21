#ifndef __FLASHMEMORY_H__
#define __FLASHMEMORY_H__

#define MODBUS_DTA9696 0x01

typedef struct modbusQuery{
	unsigned char addr;
	unsigned char reg;
    unsigned short freq; // 0.1s
    unsigned char type;
} modbusQuery;

unsigned char modbus_get_freq(unsigned char addr);
void fs_init(unsigned char* list);
void add_modbus_query(modbusQuery query);
void update_modbus_query(modbusQuery query);
int modbus_query(unsigned char addr);

#endif
