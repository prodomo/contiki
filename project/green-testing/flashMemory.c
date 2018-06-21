#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "contiki.h"
#include "cfs/cfs.h"
#include "cfs-coffee-arch.h"

#include "flashMemory.h"
#include "modbus-api.h"

static modbusQuery* query_list = NULL;
static int length = 0;

int
address_lookup(unsigned char addr) {
    for(int i = 0; i < length/sizeof(modbusQuery); i++) {
        if(query_list[i].addr == addr){
            return i;
        }
    }
    return -1;
}

unsigned char
modbus_get_freq(unsigned char addr) {
    int num = address_lookup(addr);
    if(num < 0) {
        printf("**ERR: No %x address found. \n\r", addr);
        return -1;
    }
    return query_list[num].freq;
}

void 
fs_init(unsigned char* list) {
    modbus_init();

    int fd = cfs_open("modbus", CFS_READ | CFS_WRITE | CFS_APPEND);
    if(fd < 0) {
        printf("**CFSERR: Fail to open modbus file. No data will be saved. \n\r");
        return;
    }

    length = cfs_seek(fd, 0, CFS_SEEK_END);
    if(length < 0) {
        printf("**CFSERR: Fail to seek modbus file. No data will be saved. \n\r");
        cfs_close(fd);
        return;
    } else if(length == 0) {
        printf("**CFSNOTE: No previous data. Waiting for command. \n\r");
    } else {
        query_list = malloc(length);
        cfs_seek(fd, 0, CFS_SEEK_SET);
        cfs_read(fd, query_list, length);

        printf("**CFSNOTE: Previous data read. \n\r");
        cfs_close(fd);

        for(int i = 0; i < length/sizeof(modbusQuery); i++) {
            list[i] = query_list[i].addr;
        }
    }
}

void
add_modbus_query(modbusQuery query) {
    modbusQuery* tmp = malloc(length + sizeof(modbusQuery));
    memcpy(tmp, query_list, length);
    tmp[(length/sizeof(modbusQuery))] = query;

    int fd = cfs_open("modbus", CFS_READ | CFS_WRITE);
    if(fd < 0) {
        printf("**CFSERR: Fail to open modbus file. No data will be saved. \n\r");
    } else {
        cfs_write(fd, tmp, sizeof(tmp));
        printf("**CFSNOTE: New query data saved. \n\r");
    }
    
    if(length + sizeof(modbusQuery) != sizeof(tmp)){
        printf("**CFSERR: Something wrong.... \n\r");
        return;
    }

    free(query_list);
    query_list = tmp;
}

void
update_modbus_query(modbusQuery query) {
    int num = address_lookup(query.addr);
    if(num < 0) {
        printf("**ERR: No %x address found. \n\r", query.addr);
        return;
    }

    modbusQuery* tmp = malloc(length);
    memcpy(tmp, query_list, length);

    tmp[num] = query;

    int fd = cfs_open("modbus", CFS_READ | CFS_WRITE);
    if(fd < 0) {
        printf("**CFSERR: Fail to open modbus file. No data will be saved. \n\r");
    } else {
        cfs_write(fd, tmp, sizeof(tmp));
        printf("**CFSNOTE: New query data saved. \n\r");
    }
    
    if(length + sizeof(modbusQuery) != sizeof(tmp)){
        printf("**CFSERR: Something wrong.... \n\r");
        return;
    }

    free(query_list);
    query_list = tmp;
}

int
modbus_query(unsigned char addr) {
    int num = address_lookup(addr);
    if(num < 0) {
        printf("**ERR: No %x address found. \n\r", addr);
        return -1;
    }

    int rv = modbus_read_register(addr, MODBUS_RD_HOLD_REG, query_list[num].reg, 1);
    if(rv == 0) {
        int data = modbus_get_int(0);
        printf("Success state after sending modbus packet\n\r");
        printf("Value read form %d: 0x%x = %d \n\r", addr, query_list[num].reg, data);
        return data;
    } else {
        printf("Error state after sending modbus packet: %d\n\r", rv);
        return -1;
    }
}