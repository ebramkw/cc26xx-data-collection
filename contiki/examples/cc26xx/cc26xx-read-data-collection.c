#include "contiki.h"
#include <stdio.h>
#include "dev/leds.h"

#define EXT_FLASH_BASE_ADDR 0
#define EXT_FLASH_SIZE 4*1024*1024

/*---------------------------------------------------------------------------*/
PROCESS(read_external_flash, "Read external flash");
AUTOSTART_PROCESSES(&read_external_flash);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(read_external_flash, ev, data){
	PROCESS_BEGIN();

	static int address_offset = 0;
	static int p_t_l_sensor_data_int[4];
	static int a_g_sensor_data_int[7];
	static int m_sensor_data_int[4];
	static int delimeter = -1;

	// printf("READING FROM FLASH:\n");

	int rv = ext_flash_open();

	if(!rv) {
		printf("CANNOT OPEN FLASH\n");
		ext_flash_close();
		return;
	}

	leds_on(LEDS_ALL);

	int start_time = 1533881700;
	int end_time = 1533893400;


	int pointer = EXT_FLASH_BASE_ADDR + address_offset;
	while(pointer < EXT_FLASH_SIZE){
		// read delimeter and update address
		rv = ext_flash_read(address_offset, sizeof(delimeter), (int *)&delimeter);
		address_offset += sizeof(delimeter);
		pointer = EXT_FLASH_BASE_ADDR + address_offset;

		if(delimeter == 0){
			// p t l
			rv = ext_flash_read(address_offset, sizeof(p_t_l_sensor_data_int), (int *)&p_t_l_sensor_data_int);
			// if(p_t_l_sensor_data_int[0] > start_time && p_t_l_sensor_data_int[0] < end_time)
				printf("%d,pressure,%d,temp,%d,light,%d\n", p_t_l_sensor_data_int[0], p_t_l_sensor_data_int[1], p_t_l_sensor_data_int[2], p_t_l_sensor_data_int[3]);
			address_offset += sizeof(p_t_l_sensor_data_int);
			pointer = EXT_FLASH_BASE_ADDR + address_offset;
		} else if(delimeter == 1){
			// a g
			rv = ext_flash_read(address_offset, sizeof(a_g_sensor_data_int), (int *)&a_g_sensor_data_int);
			// if(a_g_sensor_data_int[0] > start_time && a_g_sensor_data_int[0] < end_time)
				printf("%d,accel.x,%d,accel.y,%d,accel.z,%d,gyro.x,%d,gyro.y,%d,gyro.z,%d\n", a_g_sensor_data_int[0], a_g_sensor_data_int[1], a_g_sensor_data_int[2], a_g_sensor_data_int[3], a_g_sensor_data_int[4], a_g_sensor_data_int[5], a_g_sensor_data_int[6]);
			address_offset += sizeof(a_g_sensor_data_int);
			pointer = EXT_FLASH_BASE_ADDR + address_offset;
		} else if(delimeter == 2){
			// m
			rv = ext_flash_read(address_offset, sizeof(m_sensor_data_int), (int *)&m_sensor_data_int);
			// if(m_sensor_data_int[0] > start_time && m_sensor_data_int[0] < end_time)
				printf("%d,magnto.x,%d,magnto.y,%d,magnto.z,%d\n", m_sensor_data_int[0], m_sensor_data_int[1], m_sensor_data_int[2], m_sensor_data_int[3]);
			address_offset += sizeof(m_sensor_data_int);
			pointer = EXT_FLASH_BASE_ADDR + address_offset;
		} else if(delimeter == -1){
			//done
			// printf("%d done!\n", delimeter);
			break;
		}
	}
	ext_flash_close();

	// erase flash
	rv = ext_flash_open();
	if(!rv) {
		printf("CANNOT OPEN FLASH\n");
		ext_flash_close();
		return;
	}
	rv = ext_flash_erase(EXT_FLASH_BASE_ADDR, EXT_FLASH_SIZE);
	ext_flash_close();


	leds_off(LEDS_ALL);

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/