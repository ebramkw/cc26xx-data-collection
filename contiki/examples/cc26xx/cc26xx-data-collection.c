/*
 * Copyright (c) 2014, Texas Instruments Incorporated - http://www.ti.com/
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup cc26xx-platforms
 * @{
 *
 * \defgroup cc26xx-examples CC26xx Example Projects
 *
 * Example projects for CC26xx-based platforms.
 * @{
 *
 * \defgroup cc26xx-demo CC26xx Demo Project
 *
 *   Example project demonstrating the CC13xx/CC26xx platforms
 *
 *   This example will work for the following boards:
 *   - srf06-cc26xx: SmartRF06EB + CC13xx/CC26xx EM
 *   - CC2650 and CC1350 SensorTag
 *   - CC1310, CC1350, CC2650 LaunchPads
 *
 *   This is an IPv6/RPL-enabled example. Thus, if you have a border router in
 *   your installation (same RDC layer, same PAN ID and RF channel), you should
 *   be able to ping6 this demo node.
 *
 *   This example also demonstrates CC26xx BLE operation. The process starts
 *   the BLE beacon daemon (implemented in the RF driver). The daemon will
 *   send out a BLE beacon periodically. Use any BLE-enabled application (e.g.
 *   LightBlue on OS X or the TI BLE Multitool smartphone app) and after a few
 *   seconds the cc26xx device will be discovered.
 *
 * - etimer/clock : Every CC26XX_DEMO_LOOP_INTERVAL clock ticks the LED defined
 *                  as CC26XX_DEMO_LEDS_PERIODIC will toggle and the device
 *                  will print out readings from some supported sensors
 * - sensors      : Some sensortag sensors are read asynchronously (see sensor
 *                  documentation). For those, this example will print out
 *                  readings in a staggered fashion at a random interval
 * - Buttons      : CC26XX_DEMO_SENSOR_1 button will toggle CC26XX_DEMO_LEDS_BUTTON
 *                - CC26XX_DEMO_SENSOR_2 turns on LEDS_REBOOT and causes a
 *                  watchdog reboot
 *                - The remaining buttons will just print something
 *                - The example also shows how to retrieve the duration of a
 *                  button press (in ticks). The driver will generate a
 *                  sensors_changed event upon button release
 * - Reed Relay   : Will toggle the sensortag buzzer on/off
 *
 * @{
 *
 * \file
 *     Example demonstrating the cc26xx platforms
 */
#include "contiki.h"
#include "sys/etimer.h"
#include "sys/ctimer.h"
#include "dev/leds.h"
#include "dev/watchdog.h"
#include "random.h"
#include "button-sensor.h"
#include "batmon-sensor.h"
#include "board-peripherals.h"
#include "rf-core/rf-ble.h"

#include "ti-lib.h"

#include <stdio.h>
#include <stdint.h>

#include "core/net/rime/rime.h"

// #include <string.h>
// #include "cfs/cfs.h"
// #include "cfs/cfs-coffee.h"

/*---------------------------------------------------------------------------*/
#define CC26XX_DEMO_LOOP_INTERVAL       (CLOCK_SECOND * 0.05)
#define CC26XX_DEMO_LEDS_PERIODIC       LEDS_YELLOW
#define CC26XX_DEMO_LEDS_BUTTON         LEDS_RED
#define CC26XX_DEMO_LEDS_REBOOT         LEDS_ALL
/*---------------------------------------------------------------------------*/
#define CC26XX_DEMO_SENSOR_NONE         (void *)0xFFFFFFFF

#define CC26XX_DEMO_SENSOR_1     &button_left_sensor
#define CC26XX_DEMO_SENSOR_2     &button_right_sensor

#if BOARD_SENSORTAG
#define CC26XX_DEMO_SENSOR_3     CC26XX_DEMO_SENSOR_NONE
#define CC26XX_DEMO_SENSOR_4     CC26XX_DEMO_SENSOR_NONE
#define CC26XX_DEMO_SENSOR_5     &reed_relay_sensor
#elif BOARD_LAUNCHPAD
#define CC26XX_DEMO_SENSOR_3     CC26XX_DEMO_SENSOR_NONE
#define CC26XX_DEMO_SENSOR_4     CC26XX_DEMO_SENSOR_NONE
#define CC26XX_DEMO_SENSOR_5     CC26XX_DEMO_SENSOR_NONE
#else
#define CC26XX_DEMO_SENSOR_3     &button_up_sensor
#define CC26XX_DEMO_SENSOR_4     &button_down_sensor
#define CC26XX_DEMO_SENSOR_5     &button_select_sensor
#endif
/*---------------------------------------------------------------------------*/
static struct etimer et;
/*---------------------------------------------------------------------------*/
PROCESS(cc26xx_demo_process, "cc26xx demo process");
AUTOSTART_PROCESSES(&cc26xx_demo_process);
/*---------------------------------------------------------------------------*/
#if BOARD_SENSORTAG
/*---------------------------------------------------------------------------*/
/*
 * Update sensor readings in a staggered fashion every SENSOR_READING_PERIOD
 * ticks + a random interval between 0 and SENSOR_READING_RANDOM ticks
 */
#define SENSOR_READING_PERIOD (CLOCK_SECOND * 2)

static struct ctimer bmp_timer, opt_timer, hdc_timer, tmp_timer, mpu_timer;
/*---------------------------------------------------------------------------*/
static void init_bmp_reading(void *not_used);
static void init_opt_reading(void *not_used);
static void init_hdc_reading(void *not_used);
static void init_tmp_reading(void *not_used);
static void init_mpu_reading(void *not_used);
/*---------------------------------------------------------------------------*/
#define ACCEL_GYRO_SENSOR_READING_PERIOD (CLOCK_SECOND * 60)

#define MAGNTO_SENSOR_READING_PERIOD (60 * 30)

static int pressure = -1;
static int light = -1;
static int temp = -1;

struct  accel {int x; int y; int z;};
static struct accel accel_data;
struct  gyro {int x; int y; int z;};
static struct gyro gyro_data;
struct  magnto {int x; int y; int z;};
static struct magnto magnto_data;

static bool write_to_ext_flash = true;
#define EXT_FLASH_BASE_ADDR_SENSOR_DATA 0
#define EXT_FLASH_MEMORY_END_ADDRESS 0x3FFFE8 //0x400010
#define EXT_FLASH_BASE_ADDR 0
#define EXT_FLASH_SIZE 4*1024*1024

static int p_t_l_sensor_data_int[5];
static int a_g_sensor_data_int[8];
static int m_sensor_data_int[5];

static int p_t_l_sensor_data[4];
static int a_g_sensor_data[7];
static int m_sensor_data[4];

static int address_index_writing = 0;
static int start_index;

static bool time_set = false;

static bool stop_logging = false;

static int timestamp = 0;
static int temp_timestamp = 0;

static int start_time_when_get_timestamp = 0;

/*---------------------------------------------------------------------------*/
static void
broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{
  leds_on(LEDS_RED);
  // printf("broadcast message received from %d.%d: '%s'\n",
  //        from->u8[0], from->u8[1], (char *)packetbuf_dataptr());

  temp_timestamp = atoi(packetbuf_dataptr());

  // printf("%d\n", temp_timestamp);
  if(temp_timestamp == 1010101010){
    stop_logging = true;
  } else if(temp_timestamp != 0){
    timestamp = temp_timestamp;
    time_set = true;
    stop_logging = false;
    start_time_when_get_timestamp = clock_seconds();
  }
  leds_off(LEDS_RED);
}
static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
static struct broadcast_conn broadcast;
/*---------------------------------------------------------------------------*/
static void
print_mpu_reading(int reading)
{
  if(reading < 0) {
    printf("-");
    reading = -reading;
  }

  printf("%d.%02d", reading / 100, reading % 100);
}
/*---------------------------------------------------------------------------*/
static void
get_bmp_reading()
{
  int value;
  clock_time_t next = SENSOR_READING_PERIOD;

  value = bmp_280_sensor.value(BMP_280_SENSOR_TYPE_PRESS);
  if(value != CC26XX_SENSOR_READING_ERROR) {
    // printf("BAR: Pressure=%d.%02d hPa\n", value / 100, value % 100);
    pressure = value;
  } else {
    printf("BAR: Pressure Read Error\n");
    pressure = -1;
  }

  SENSORS_DEACTIVATE(bmp_280_sensor);

  ctimer_set(&bmp_timer, next, init_bmp_reading, NULL);
}
/*---------------------------------------------------------------------------*/
static void
get_tmp_reading()
{
  int value;
  clock_time_t next = SENSOR_READING_PERIOD;

  value = tmp_007_sensor.value(TMP_007_SENSOR_TYPE_ALL);

  if(value != CC26XX_SENSOR_READING_ERROR) {
    value = tmp_007_sensor.value(TMP_007_SENSOR_TYPE_AMBIENT);
    temp = value;
    // printf("TMP: Ambient=%d.%03d C\n", value / 1000, value % 1000);
  }else {
    printf("TMP: Ambient Read Error\n");
    temp = -1;
  }

  SENSORS_DEACTIVATE(tmp_007_sensor);

  ctimer_set(&tmp_timer, next, init_tmp_reading, NULL);
}
/*---------------------------------------------------------------------------*/
static void
get_light_reading()
{
  int value;
  clock_time_t next = SENSOR_READING_PERIOD;

  value = opt_3001_sensor.value(0);
  if(value != CC26XX_SENSOR_READING_ERROR) {
    // printf("OPT: Light=%d.%02d lux\n", value / 100, value % 100);
    light = value;
  } else {
    printf("OPT: Light Read Error\n");
    light = -1;
  }

  /* The OPT will turn itself off, so we don't need to call its DEACTIVATE */
  ctimer_set(&opt_timer, next, init_opt_reading, NULL);
}
/*---------------------------------------------------------------------------*/

static int counter = 0;
static int start = 0;
static int accel_gyro_enabled = 0;
static int magnto_enabled = 0;
static int last_magnto_read = -2 * MAGNTO_SENSOR_READING_PERIOD;

static void
get_mpu_reading()
{
  gyro_data.x = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_X);
  gyro_data.y = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Y);
  gyro_data.z = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Z);

  accel_data.x = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_X);
  accel_data.y = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Y);
  accel_data.z = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Z);

  if(clock_seconds() - last_magnto_read >= MAGNTO_SENSOR_READING_PERIOD){
    magnto_enabled = 1;

    magnto_data.x = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_MAG_X);
    magnto_data.y = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_MAG_Y);
    magnto_data.z = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_MAG_Z);
  }

  clock_time_t next;

  if (counter == 0){
    accel_gyro_enabled = 1;
    start = clock_seconds();
  }

  if(clock_seconds() - start < 5){ // 20 value in a second for 5 seconds
    counter++;
    init_mpu_reading(NULL);
  } else{
    if(clock_seconds() - last_magnto_read >= MAGNTO_SENSOR_READING_PERIOD){
      // printf("\nMagnto included: ");
      last_magnto_read = clock_seconds();
      magnto_enabled = 0;
    }

    accel_gyro_enabled = 0;
    // printf("start = %d, end = %d with counter = %d\n", start, clock_seconds(), counter);
    next = ACCEL_GYRO_SENSOR_READING_PERIOD;
    counter = 0;
    SENSORS_DEACTIVATE(mpu_9250_sensor);
    ctimer_set(&mpu_timer, next, init_mpu_reading, NULL);
  }
}
/*---------------------------------------------------------------------------*/
static void
init_bmp_reading(void *not_used)
{
  SENSORS_ACTIVATE(bmp_280_sensor);
}
/*---------------------------------------------------------------------------*/
static void
init_opt_reading(void *not_used)
{
  SENSORS_ACTIVATE(opt_3001_sensor);
}
/*---------------------------------------------------------------------------*/
static void
init_hdc_reading(void *not_used)
{
  SENSORS_ACTIVATE(hdc_1000_sensor);
}
/*---------------------------------------------------------------------------*/
static void
init_tmp_reading(void *not_used)
{
  SENSORS_ACTIVATE(tmp_007_sensor);
}
/*---------------------------------------------------------------------------*/
static void
init_mpu_reading(void *not_used)
{
  int readings_bitmap = 0;
  readings_bitmap |= MPU_9250_SENSOR_TYPE_ACC;
  readings_bitmap |= MPU_9250_SENSOR_TYPE_GYRO;

  mpu_9250_sensor.configure(SENSORS_ACTIVE, readings_bitmap);
}
#endif
/*---------------------------------------------------------------------------*/
static void
get_sync_sensor_readings(void)
{
  int value;

  // printf("-----------------------------------------\n");

  // value = batmon_sensor.value(BATMON_SENSOR_TYPE_TEMP);
  // printf("Bat: Temp=%d C\n", value);

  // value = batmon_sensor.value(BATMON_SENSOR_TYPE_VOLT);
  // printf("Bat: Volt=%d mV\n", (value * 125) >> 5);

#if BOARD_SMARTRF06EB
  SENSORS_ACTIVATE(als_sensor);

  value = als_sensor.value(0);
  printf("ALS: %d raw\n", value);

  SENSORS_DEACTIVATE(als_sensor);
#endif

  return;
}
/*---------------------------------------------------------------------------*/
static void
init_sensors(void)
{
#if BOARD_SENSORTAG
  SENSORS_ACTIVATE(reed_relay_sensor);
#endif

  SENSORS_ACTIVATE(batmon_sensor);
}
/*---------------------------------------------------------------------------*/
static void
init_sensor_readings(void)
{
#if BOARD_SENSORTAG
  SENSORS_ACTIVATE(hdc_1000_sensor);
  SENSORS_ACTIVATE(tmp_007_sensor);
  SENSORS_ACTIVATE(opt_3001_sensor);
  SENSORS_ACTIVATE(bmp_280_sensor);

  init_mpu_reading(NULL);
#endif
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(cc26xx_demo_process, ev, data)
{

  PROCESS_EXITHANDLER(broadcast_close(&broadcast);)

  PROCESS_BEGIN();

  broadcast_open(&broadcast, 129, &broadcast_call);

  printf("CC26XX data collection\n");

  // erase flash
  // printf("ERASING FLASH: from %d, size of %d\n", EXT_FLASH_BASE_ADDR, EXT_FLASH_SIZE);
  // leds_on(LEDS_ALL);
  // int rv = ext_flash_open();
  // if(!rv) {
  //   printf("CANNOT OPEN FLASH\n");
  //   ext_flash_close();
  //   return;
  // }
  // rv = ext_flash_erase(EXT_FLASH_BASE_ADDR, EXT_FLASH_SIZE);
  // ext_flash_close();
  // printf("DONE! \n");
  // leds_off(LEDS_ALL);

  // edit start to where we did stop
  int address_offset = 0;
  int pointer = EXT_FLASH_BASE_ADDR + address_offset;
  int delimeter = -1;
  
  int rv = ext_flash_open();

  if(!rv) {
    printf("CANNOT OPEN FLASH\n");
    ext_flash_close();
    return;
  }

  while(pointer < EXT_FLASH_SIZE){
    // read delimeter and update address
    rv = ext_flash_read(address_offset, sizeof(delimeter), (int *)&delimeter);
    address_offset += sizeof(delimeter);
    pointer = EXT_FLASH_BASE_ADDR + address_offset;

    if(delimeter == 0){
      // p t l
      rv = ext_flash_read(address_offset, sizeof(p_t_l_sensor_data), (int *)&p_t_l_sensor_data);
      address_offset += sizeof(p_t_l_sensor_data);
      pointer = EXT_FLASH_BASE_ADDR + address_offset;
      address_index_writing = pointer;
    } else if(delimeter == 1){
      // a g
      rv = ext_flash_read(address_offset, sizeof(a_g_sensor_data), (int *)&a_g_sensor_data);
      address_offset += sizeof(a_g_sensor_data);
      pointer = EXT_FLASH_BASE_ADDR + address_offset;
      address_index_writing = pointer;
    } else if(delimeter == 2){
      // m
      rv = ext_flash_read(address_offset, sizeof(m_sensor_data), (int *)&m_sensor_data);
      address_offset += sizeof(m_sensor_data);
      pointer = EXT_FLASH_BASE_ADDR + address_offset;
      address_index_writing = pointer;
    } else if(delimeter == -1){
      //done
      printf("start = @0x%08X\n", address_index_writing);
      printf("done!\n");
      break;
    }
  }
  ext_flash_close();
  start_index = address_index_writing;


  init_sensors();

  /* Init the BLE advertisement daemon */
  rf_ble_beacond_config(0, BOARD_STRING);
  rf_ble_beacond_start();

  etimer_set(&et, CC26XX_DEMO_LOOP_INTERVAL);
  // get_sync_sensor_readings();
  init_sensor_readings();

  int fd_write;
  int n;
  char *filename = "collected_data";

  while(1) {

    PROCESS_YIELD();

    if(ev == PROCESS_EVENT_TIMER) {
      if(data == &et) {
        // leds_toggle(CC26XX_DEMO_LEDS_PERIODIC);

        // get_sync_sensor_readings();

        if(!write_to_ext_flash){
          //printf("long press on the left button to start writing to the flash!\n");
          leds_on(LEDS_YELLOW);
        } else{
          leds_off(LEDS_YELLOW);
        }

        //
        if(pressure != -1 && temp != -1 && light != -1){
          // printf("%d: ", clock_seconds());
          // printf("Pressure = %d.%02d", pressure / 100, pressure % 100);
          // printf(", Temp = %d.%03d", temp / 1000, temp % 1000);
          // printf(", Light = %d.%02d\n", light / 100, light % 100);

          if(write_to_ext_flash && time_set && !stop_logging){
            leds_on(LEDS_YELLOW);

            // using external flash
            if(EXT_FLASH_BASE_ADDR_SENSOR_DATA + address_index_writing + sizeof(p_t_l_sensor_data_int) < EXT_FLASH_MEMORY_END_ADDRESS){
              int rv = ext_flash_open();
              if(!rv) {
                printf("CANNOT OPEN FLASH\n");
                ext_flash_close();
                return 0;
              }
              // printf("start = @0x%08X, WRITING TO EXTERNAL FLASH @0x%08X sizeof: %d\n", start_index, EXT_FLASH_BASE_ADDR_SENSOR_DATA + address_index_writing, sizeof(p_t_l_sensor_data_int));
              p_t_l_sensor_data_int[0] = 0;
              p_t_l_sensor_data_int[1] = timestamp + clock_seconds() - start_time_when_get_timestamp;
              p_t_l_sensor_data_int[2] = pressure;
              p_t_l_sensor_data_int[3] = temp;
              p_t_l_sensor_data_int[4] = light;
              // printf("%d %d %d %d %d\n", p_t_l_sensor_data_int[0], p_t_l_sensor_data_int[1], p_t_l_sensor_data_int[2], p_t_l_sensor_data_int[3], p_t_l_sensor_data_int[4]);

              rv = ext_flash_write(EXT_FLASH_BASE_ADDR_SENSOR_DATA + address_index_writing, sizeof(p_t_l_sensor_data_int), (int *)&p_t_l_sensor_data_int);
              if(!rv) {
                printf("Error saving data to EXT flash\n");
              }
              address_index_writing += sizeof(p_t_l_sensor_data_int);
              ext_flash_close();
            } else{
              printf("POSSIBLE EXTERNAL FLASH OVERFLOW @0x%08X sizeof: %d\n",EXT_FLASH_BASE_ADDR_SENSOR_DATA + address_index_writing, sizeof(light));
              leds_on(LEDS_RED);
            }
            leds_off(LEDS_YELLOW);
          }

          // if(write_to_ext_flash){
          //   leds_on(LEDS_YELLOW);

            // fd_write = cfs_open(filename, CFS_WRITE | CFS_APPEND);

          //   if(fd_write != -1) {
          //     p_t_l_sensor_data_int[0] = 0;
          //     p_t_l_sensor_data_int[1] = clock_seconds();
          //     p_t_l_sensor_data_int[2] = pressure;
          //     p_t_l_sensor_data_int[3] = temp;
          //     p_t_l_sensor_data_int[4] = light;

          //     n = cfs_write(fd_write, p_t_l_sensor_data_int, (int)sizeof(p_t_l_sensor_data_int));
          //     cfs_close(fd_write);
          //     printf("step 4: successfully appended data to cfs. wrote %i bytes\n", n);
          //   } else {
          //     printf("ERROR: could not write to memory.\n");
          //     leds_on(LEDS_RED);
          //   }
          //   leds_off(LEDS_YELLOW);
          // }

          pressure = -1;
          temp = -1;
          light = -1;
        }


        if(accel_gyro_enabled){
          // get reading
          get_mpu_reading();

          //print
          // printf("Acc: x = ");
          // print_mpu_reading(accel_data.x);
          // printf(", y = ");
          // print_mpu_reading(accel_data.y);
          // printf(", z = ");
          // print_mpu_reading(accel_data.z);

          // printf(" and Gyro: x = ");
          // print_mpu_reading(gyro_data.x);
          // printf(", y = ");
          // print_mpu_reading(gyro_data.y);
          // printf(", z = ");
          // print_mpu_reading(gyro_data.z);
          // printf("\n");

          if(write_to_ext_flash && time_set && !stop_logging){
            leds_on(LEDS_YELLOW);

            // using external flash
            if(EXT_FLASH_BASE_ADDR_SENSOR_DATA + address_index_writing + sizeof(a_g_sensor_data_int) < EXT_FLASH_MEMORY_END_ADDRESS){
              int rv = ext_flash_open();
              if(!rv) {
                printf("CANNOT OPEN FLASH\n");
                ext_flash_close();
                return 0;
              }
              // printf("WRITING TO EXTERNAL FLASH @0x%08X sizeof: %d\n",EXT_FLASH_BASE_ADDR_SENSOR_DATA + address_index_writing, sizeof(a_g_sensor_data_int));
              a_g_sensor_data_int[0] = 1;
              a_g_sensor_data_int[1] = timestamp + clock_seconds() - start_time_when_get_timestamp;
              a_g_sensor_data_int[2] = accel_data.x;
              a_g_sensor_data_int[3] = accel_data.y;
              a_g_sensor_data_int[4] = accel_data.z;
              a_g_sensor_data_int[5] = gyro_data.x;
              a_g_sensor_data_int[6] = gyro_data.y;
              a_g_sensor_data_int[7] = gyro_data.z;
              // printf("%d %d %d %d %d %d %d %d\n", a_g_sensor_data_int[0], a_g_sensor_data_int[1], a_g_sensor_data_int[2], a_g_sensor_data_int[3], a_g_sensor_data_int[4], a_g_sensor_data_int[5], a_g_sensor_data_int[6], a_g_sensor_data_int[7]);

              rv = ext_flash_write(EXT_FLASH_BASE_ADDR_SENSOR_DATA + address_index_writing, sizeof(a_g_sensor_data_int), (int *)&a_g_sensor_data_int);
              if(!rv) {
                printf("Error saving data to EXT flash\n");
              }
              address_index_writing += sizeof(a_g_sensor_data_int);
              ext_flash_close();
            } else{
              printf("POSSIBLE EXTERNAL FLASH OVERFLOW @0x%08X sizeof: %d\n",EXT_FLASH_BASE_ADDR_SENSOR_DATA + address_index_writing, sizeof(light));
              leds_on(LEDS_RED);
            }
            leds_off(LEDS_YELLOW);
          }

          // if(write_to_ext_flash){
          //   leds_on(LEDS_YELLOW);

          //   fd_write = cfs_open(filename, CFS_WRITE | CFS_APPEND);

          //   if(fd_write != -1) {
          //     a_g_sensor_data_int[0] = 1;
          //     a_g_sensor_data_int[1] = clock_seconds();
          //     a_g_sensor_data_int[2] = accel_data.x;
          //     a_g_sensor_data_int[3] = accel_data.y;
          //     a_g_sensor_data_int[4] = accel_data.z;
          //     a_g_sensor_data_int[5] = gyro_data.x;
          //     a_g_sensor_data_int[6] = gyro_data.y;
          //     a_g_sensor_data_int[7] = gyro_data.z;

          //     n = cfs_write(fd_write, a_g_sensor_data_int, (int)sizeof(a_g_sensor_data_int));
          //     cfs_close(fd_write);
          //     printf("step 4: successfully appended data to cfs. wrote %i bytes\n", n);
          //   } else {
          //     printf("ERROR: could not write to memory.\n");
          //     leds_on(LEDS_RED);
          //   }
          //   leds_off(LEDS_YELLOW);
          // }

          if(magnto_enabled){
            //got reading with accel and gyro => print
            // printf("Magnto: x = ");
            // print_mpu_reading(magnto_data.x);
            // printf(", y = ");
            // print_mpu_reading(magnto_data.y);
            // printf(", z = ");
            // print_mpu_reading(magnto_data.z);
            // printf("\n");

            if(write_to_ext_flash && time_set && !stop_logging){
            leds_on(LEDS_YELLOW);

            // using external flash
            if(EXT_FLASH_BASE_ADDR_SENSOR_DATA + address_index_writing + sizeof(m_sensor_data_int) < EXT_FLASH_MEMORY_END_ADDRESS){
              int rv = ext_flash_open();
              if(!rv) {
                printf("CANNOT OPEN FLASH\n");
                ext_flash_close();
                return 0;
              }
              // printf("WRITING TO EXTERNAL FLASH @0x%08X sizeof: %d\n",EXT_FLASH_BASE_ADDR_SENSOR_DATA + address_index_writing, sizeof(m_sensor_data_int));
              m_sensor_data_int[0] = 2;
              m_sensor_data_int[1] = timestamp + clock_seconds() - start_time_when_get_timestamp;
              m_sensor_data_int[2] = magnto_data.x;
              m_sensor_data_int[3] = magnto_data.y;
              m_sensor_data_int[4] = magnto_data.z;
              // printf("%d %d %d %d %d\n", m_sensor_data_int[0], m_sensor_data_int[1], m_sensor_data_int[2], m_sensor_data_int[3], m_sensor_data_int[4]);

              rv = ext_flash_write(EXT_FLASH_BASE_ADDR_SENSOR_DATA + address_index_writing, sizeof(m_sensor_data_int), (int *)&m_sensor_data_int);
              if(!rv) {
                printf("Error saving data to EXT flash\n");
              }
              address_index_writing += sizeof(m_sensor_data_int);
              ext_flash_close();
              } else{
                printf("POSSIBLE EXTERNAL FLASH OVERFLOW @0x%08X sizeof: %d\n",EXT_FLASH_BASE_ADDR_SENSOR_DATA + address_index_writing, sizeof(light));
                leds_on(LEDS_RED);
              }
              leds_off(LEDS_YELLOW);
            }

            // if(write_to_ext_flash){
            //   leds_on(LEDS_YELLOW);

            //   fd_write = cfs_open(filename, CFS_WRITE | CFS_APPEND);

            //   if(fd_write != -1) {
            //     m_sensor_data_int[0] = 2;
            //     m_sensor_data_int[1] = clock_seconds();
            //     m_sensor_data_int[2] = magnto_data.x;
            //     m_sensor_data_int[3] = magnto_data.y;
            //     m_sensor_data_int[4] = magnto_data.z;

            //     n = cfs_write(fd_write, m_sensor_data_int, (int)sizeof(m_sensor_data_int));
            //     cfs_close(fd_write);
            //     printf("step 4: successfully appended data to cfs. wrote %i bytes\n", n);
            //   } else {
            //     printf("ERROR: could not write to memory.\n");
            //     leds_on(LEDS_RED);
            //   }
            //   leds_off(LEDS_YELLOW);
            // }
          }
        }
        //

        etimer_set(&et, CC26XX_DEMO_LOOP_INTERVAL);
      }
    } else if(ev == sensors_event) {
      if(data == CC26XX_DEMO_SENSOR_1) {
        printf("Left: Pin %d, press duration %d clock ticks\n",
               (CC26XX_DEMO_SENSOR_1)->value(BUTTON_SENSOR_VALUE_STATE),
               (CC26XX_DEMO_SENSOR_1)->value(BUTTON_SENSOR_VALUE_DURATION));

        if((CC26XX_DEMO_SENSOR_1)->value(BUTTON_SENSOR_VALUE_DURATION) >
           CLOCK_SECOND) {
          printf("Long button press!\n");
          if(!write_to_ext_flash){
              write_to_ext_flash = true;
              printf("WILL NOW WRITE TO FLASH!\n");
            }
        }

        // leds_toggle(CC26XX_DEMO_LEDS_BUTTON);
      } else if(data == CC26XX_DEMO_SENSOR_2) {
        if((CC26XX_DEMO_SENSOR_2)->value(BUTTON_SENSOR_VALUE_DURATION) >
           CLOCK_SECOND) {
          leds_on(CC26XX_DEMO_LEDS_REBOOT);
          watchdog_reboot();
        }
      } else if(data == CC26XX_DEMO_SENSOR_3) {
        printf("Up\n");
      } else if(data == CC26XX_DEMO_SENSOR_4) {
        printf("Down\n");
      } else if(data == CC26XX_DEMO_SENSOR_5) {
#if BOARD_SENSORTAG
        if(buzzer_state()) {
          buzzer_stop();
        } else {
          buzzer_start(1000);
        }
      } else if(ev == sensors_event && data == &bmp_280_sensor) {
        get_bmp_reading();
      } else if(ev == sensors_event && data == &opt_3001_sensor) {
        get_light_reading();
      } else if(ev == sensors_event && data == &tmp_007_sensor) {
        get_tmp_reading();
      } else if(ev == sensors_event && data == &mpu_9250_sensor) {
        get_mpu_reading();
#elif BOARD_SMARTRF06EB
        printf("Sel: Pin %d, press duration %d clock ticks\n",
               button_select_sensor.value(BUTTON_SENSOR_VALUE_STATE),
               button_select_sensor.value(BUTTON_SENSOR_VALUE_DURATION));
#endif
      }
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 * @}
 */
