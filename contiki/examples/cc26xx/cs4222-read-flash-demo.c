#include "contiki.h"
#include <stdio.h>

#define EXT_FLASH_BASE_ADDR 0
#define EXT_FLASH_SIZE 4*1024*1024
/*---------------------------------------------------------------------------*/
PROCESS(read_external_flash, "Read external flash");
AUTOSTART_PROCESSES(&read_external_flash);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(read_external_flash, ev, data){
PROCESS_BEGIN();

static int address_offset = 0;
static int sensor_data_int[1];

printf("READING FROM FLASH:\n");

int rv = ext_flash_open();

if(!rv) {
printf("CANNOT OPEN FLASH\n");
ext_flash_close();
return;
}

int pointer = EXT_FLASH_BASE_ADDR + address_offset;
while(pointer < EXT_FLASH_SIZE){
rv = ext_flash_read(address_offset, sizeof(sensor_data_int), (int *)&sensor_data_int);
printf("%08X,%d\n", pointer,sensor_data_int[0]);
address_offset += sizeof(sensor_data_int);
pointer = EXT_FLASH_BASE_ADDR + address_offset;
}
ext_flash_close();
PROCESS_END();
}
/*---------------------------------------------------------------------------*/