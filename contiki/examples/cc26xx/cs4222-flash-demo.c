/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "dev/leds.h"
#include "button-sensor.h"
#include "board-peripherals.h"
#include "node-id.h"
#include <stdio.h>
/*---------------------------------------------------------------------------*/
#define CC26XX_DEMO_LOOP_INTERVAL (CLOCK_SECOND * 2) // sample every 2 seconds (0.5 HZ)
#define CC26XX_DEMO_SENSOR_1 &button_left_sensor
#define EXT_FLASH_BASE_ADDR_SENSOR_DATA 0
#define EXT_FLASH_MEMORY_END_ADDRESS 0x400010
#define EXT_FLASH_BASE_ADDR 0
#define EXT_FLASH_SIZE 4*1024*1024
#define version                     "1.0"
/*---------------------------------------------------------------------------*/
static struct etimer et, et_sensor_read, et_blink;
static bool write_to_ext_flash = false;
static uint8_t blinks;
static int light = -1;
static int temp = -1;
static int pressure = -1;
struct accel{ int x; int y; int z;};
static struct accel accel_data;
struct gyro{ int x; int y; int z;};
static struct gyro gyro_data;
struct  magnto{ int x; int y; int z;};
static struct magnto magnto_data;
static int address_index_writing = -sizeof(light);
/*---------------------------------------------------------------------------*/
PROCESS(cs4222_flash_demo_process, "PROCESS: Flashing external memory with data from light sensor");
PROCESS(blink_process, "PROCESS: LED blink for long press");
AUTOSTART_PROCESSES(&cs4222_flash_demo_process, &blink_process);
/*---------------------------------------------------------------------------*/
static int get_light_reading(){
	int value;
	value = opt_3001_sensor.value(0);
	if(value != CC26XX_SENSOR_READING_ERROR) {
		SENSORS_ACTIVATE(opt_3001_sensor);
		return value;
	} else {
		printf("OPT: Light Read Error\n");
		return -1;
	}
}

static int get_temp_reading(){
	int value;
	value = tmp_007_sensor.value(TMP_007_SENSOR_TYPE_ALL);

	if(value == CC26XX_SENSOR_READING_ERROR) {
		printf("TMP: Ambient Read Error\n");
		return -1;
	}

	value = tmp_007_sensor.value(TMP_007_SENSOR_TYPE_AMBIENT);
	SENSORS_ACTIVATE(tmp_007_sensor);
	return value;
}

static int get_bmp_reading()
{
  int value;
  value = bmp_280_sensor.value(BMP_280_SENSOR_TYPE_PRESS);
  if(value != CC26XX_SENSOR_READING_ERROR) {
  	SENSORS_ACTIVATE(bmp_280_sensor);
    return value;
  } else {
    printf("BAR: Pressure Read Error\n");
    return -1;
  }
}

static void get_mpu_reading()
{  
  gyro_data.x = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_X);
  gyro_data.y = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Y);
  gyro_data.z = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Z);

  accel_data.x = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_X);
  accel_data.y = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Y);
  accel_data.z = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Z);

  magnto_data.x = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_MAG_X);
  magnto_data.y = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_MAG_Y);
  magnto_data.z = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_MAG_Z);
}

static void print_mpu_reading(int reading)
{
  if(reading < 0) {
    printf("-");
    reading = -reading;
  }

  printf("%d.%02d", reading / 100, reading % 100);
}

static void print_logo(){
	printf("\n*************************************************************************\n");
	printf("CS4222 DEMO: WRITING OPTICAL SENSOR DATA TO FLASH MEMORY , ver %s\n", version);
	printf("*************************************************************************\n");
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(cs4222_flash_demo_process, ev, data){
	PROCESS_BEGIN();
	etimer_set(&et, CC26XX_DEMO_LOOP_INTERVAL);
	print_logo();
	
	// activate sensors
	SENSORS_ACTIVATE(opt_3001_sensor);
	SENSORS_ACTIVATE(tmp_007_sensor);
	SENSORS_ACTIVATE(bmp_280_sensor);
	mpu_9250_sensor.configure(SENSORS_ACTIVE, MPU_9250_SENSOR_TYPE_ALL);
	
	while(1) {

		PROCESS_WAIT_EVENT();

		if(ev == PROCESS_EVENT_TIMER) {
			if(data == &et_sensor_read) {

			}
			if(data == &et) {
				printf("nodeid,%d,light = %d.%02d, temp = %d.%03d, Pressure = %d.%02d", node_id, light/100, light%100, temp/1000, temp%1000, pressure / 100, pressure % 100);

				printf(", accel.x = ");
				print_mpu_reading(accel_data.x);
				printf(", accel.y = ");
				print_mpu_reading(accel_data.y);
				printf(", accel.z = ");
				print_mpu_reading(accel_data.z);

				printf(", gyro.x = ");
				print_mpu_reading(gyro_data.x);
				printf(", gyro.y = ");
				print_mpu_reading(gyro_data.y);
				printf(", gyro.z = ");
				print_mpu_reading(gyro_data.z);

				printf(", magnto.x = ");
				print_mpu_reading(magnto_data.x);
				printf(", magnto.y = ");
				print_mpu_reading(magnto_data.y);
				printf(", magnto.z = ");
				print_mpu_reading(magnto_data.z);

				printf("\n");

				if(write_to_ext_flash){
					leds_on(LEDS_YELLOW);

					// using external flash
					if(EXT_FLASH_BASE_ADDR_SENSOR_DATA + address_index_writing + sizeof(light) < EXT_FLASH_MEMORY_END_ADDRESS){
						int rv = ext_flash_open();
						if(!rv) {
							printf("CANNOT OPEN FLASH\n");
							ext_flash_close();
							return 0;
						}
						printf("WRITING TO EXTERNAL FLASH @0x%08X sizeof: %d\n",EXT_FLASH_BASE_ADDR_SENSOR_DATA + address_index_writing, sizeof(light));
						printf("%d\n", light);
						rv = ext_flash_write(EXT_FLASH_BASE_ADDR_SENSOR_DATA + address_index_writing, sizeof(light), (uint8_t *)&light);
						if(!rv) {
							printf("Error saving data to EXT flash\n");
						}
						address_index_writing += sizeof(light);
						ext_flash_close();
					} else{
						printf("POSSIBLE EXTERNAL FLASH OVERFLOW @0x%08X sizeof: %d\n",EXT_FLASH_BASE_ADDR_SENSOR_DATA + address_index_writing, sizeof(light));
					}
					leds_off(LEDS_YELLOW);
				}
				etimer_set(&et, CC26XX_DEMO_LOOP_INTERVAL);
			}
		} else if(ev == sensors_event) {
			if(data == CC26XX_DEMO_SENSOR_1) {
				printf("Left: Pin %d, press duration %d clock ticks\n",
				(CC26XX_DEMO_SENSOR_1)->value(BUTTON_SENSOR_VALUE_STATE),
				(CC26XX_DEMO_SENSOR_1)->value(BUTTON_SENSOR_VALUE_DURATION));

				if((CC26XX_DEMO_SENSOR_1)->value(BUTTON_SENSOR_VALUE_DURATION) > CLOCK_SECOND) {
					printf("Long button press!\n");
					if(!write_to_ext_flash){
						write_to_ext_flash=true;
						printf("WILL NOW WRITE TO FLASH!\n");
					}
				}
			} else if(ev == sensors_event && data == &opt_3001_sensor) {
				light = get_light_reading();
			} else if(ev == sensors_event && data == &tmp_007_sensor) {
				temp = get_temp_reading();
			} else if(ev == sensors_event && data == &bmp_280_sensor) {
				pressure = get_bmp_reading();
			} else if(ev == sensors_event && data == &mpu_9250_sensor) {
				get_mpu_reading();
    		}
		}
	}
	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(blink_process, ev, data){
	PROCESS_BEGIN();
	blinks = 0;
	while(1) {
		etimer_set(&et_blink, CLOCK_SECOND*2);

		if(!write_to_ext_flash){
			print_logo();
			printf("Long press (over 1 second) left pin to write sensor data!\n");
			printf("RED LED blinks 4 times when writing to flash is activated!\n");
			printf("*************************************************************************\n");
		}
		PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);
		if(blinks < 8 && write_to_ext_flash){
			leds_toggle(LEDS_RED);
			blinks++;
		} else{
			if(blinks == 8){
				leds_off(LEDS_RED);
			    PROCESS_EXIT();
			} else{
			    etimer_set(&et_blink, CLOCK_SECOND);    
			}
		}
	}
	PROCESS_END();
}
/*---------------------------------------------------------------------------*/