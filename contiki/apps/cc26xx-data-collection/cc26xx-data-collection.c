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

#include "cc26xx-data-collection.h"
#include "defs_and_types.h"

/*---------------------------------------------------------------------------*/
PROCESS(cc26xx_data_collection_process, "cc26xx data collection process");
PROCESS(sync_data, "sync data");
AUTOSTART_PROCESSES(&cc26xx_data_collection_process);
/*---------------------------------------------------------------------------*/
int cc26xx_data_collection_loop_interval = ACCEL_GYRO_MAGNTO_SAMPLING_PERIOD;
static struct etimer et;
/*---------------------------------------------------------------------------*/
// timer to wait for node to start sink before claim its permission
static struct ctimer sink_busy_timer_before, sink_busy_timer_after;
static bool start_sync = false;
static bool sink_busy = false;
static unsigned short busy_node_id = 0;
static bool busy_node_start = false;
/*---------------------------------------------------------------------------*/
// for sink, timestamp got from serial port
// for nodes, timestamp got from sink
static bool time_set = false;
static int timestamp = 0;
static int start_time_when_get_timestamp = 0;
/*---------------------------------------------------------------------------*/
static data_packet_struct received_packet;
static data_packet_struct data_packet;
/*---------------------------------------------------------------------------*/
// mapping between node id and object to which the node is attached
static const uint16_t mapping_id[] = NODE_ID_MAPPING;
static const char* mapping_object[] = NODE_OBJECT_MAPPING;
/*---------------------------------------------------------------------------*/
// keep track of total recorded size
static bool write_to_ext_flash = false;
int total_size = 0;
/*---------------------------------------------------------------------------*/
static bool execute_once_flag = true;
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
static char* get_mapping_object(uint16_t nodeID){
  uint8_t i = 0;
  for(i = 0; i < DATA_COLLECTION_NODES; i++)
    if(mapping_id[i] == nodeID)
      return (char*) mapping_object[i];
  return "";
}
/*---------------------------------------------------------------------------*/
static void
broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{
  leds_on(LEDS_RED);
  // printf("broadcast message received from %d.%d: '%s'\n",
  //        from->u8[0], from->u8[1], (char *)packetbuf_dataptr());

  memcpy(&received_packet, packetbuf_dataptr(), sizeof(data_packet_struct));

  if(node_id != SINK_ID && received_packet.src_id == SINK_ID){
    // printf("received from sink, ");
    // printf("flag = %d and node_id = %d\n", received_packet.flag, received_packet.dst_id);
    if(received_packet.dst_id == node_id){
      if(received_packet.flag == 1){
        /*start to sync, set flag*/
        start_sync = true;
      } else if(received_packet.flag == 2) {
        /*timestamp, set it and set flag*/
        timestamp = received_packet.timestamp;
        time_set = true;
        start_time_when_get_timestamp = clock_seconds();
      }
    }
  } else if(node_id == SINK_ID && received_packet.dst_id == SINK_ID){
    // sink
    // printf("received from a node, ");
    // received_packet = (data_packet_struct*)(packetbuf_dataptr());
    // printf("flag = %d and node_id = %d\n", received_packet.flag, received_packet.src_id);
    if(received_packet.flag == 1){
      /*ask to sink*/
      /*if not busy, send start to sync and set flag*/
      if(!sink_busy){
        sink_busy = true;
        busy_node_id = received_packet.src_id;
        // set a timer call back to free busy after 1 minute if node does not start to send data
        busy_node_start = false;
        ctimer_set(&sink_busy_timer_before, FREE_SINK_NOT_STARTED_DURATION, free_sink_busy_before, NULL);

        execute_once_flag = true; // for printing
        // printf("flag 1\n");
        process_post(&cc26xx_data_collection_process, PROCESS_EVENT_CONTINUE, NULL);
      }
    } else if(received_packet.flag == 2){
      // printf("flag 2\n");
      // free busy if done
      if(received_packet.src_id == busy_node_id){
        sink_busy = false;
        busy_node_id = 0;

        ctimer_stop(&sink_busy_timer_after);
      }
      process_post(&cc26xx_data_collection_process, PROCESS_EVENT_POLL, NULL);
    } else if(received_packet.flag == 3 && sink_busy && received_packet.src_id == busy_node_id){
      if(!busy_node_start){
        busy_node_start = true;

        ctimer_stop(&sink_busy_timer_before);

        ctimer_set(&sink_busy_timer_after, FREE_SINK_NOT_FINISHED_DURATION, free_sink_busy_after, NULL);
      }
      if(execute_once_flag){
        execute_once_flag = false;
        timestamp = timestamp + clock_seconds() - start_time_when_get_timestamp;        
        // printf("time when node start to sync = %d\n", timestamp);
      }
      // data
      printf("\"%s\":{\"timestamp\":\"%d\",", get_mapping_object(busy_node_id), received_packet.timestamp);
      if(received_packet.delimiter == SENSOR_TYPE.PRESSURE_TEMP_LIGHT)
        printf("\"pressure\":\"%d\",\"temp\":\"%d\",\"light\":\"%d\"},\n", received_packet.v1, received_packet.v2, received_packet.v3);
      else if(received_packet.delimiter == SENSOR_TYPE.ACCEL)
        printf("\"accel\":{\"x\":\"%d\",\"y\":\"%d\",\"z\":\"%d\"}},\n", received_packet.v1, received_packet.v2, received_packet.v3);
      else if(received_packet.delimiter == SENSOR_TYPE.GYRO)
        printf("\"gyro\":{\"x\":\"%d\",\"y\":\"%d\",\"z\":\"%d\"}},\n", received_packet.v1, received_packet.v2, received_packet.v3);
      else if(received_packet.delimiter == SENSOR_TYPE.MAGNTO)
        printf("\"magnto\":{\"x\":\"%d\",\"y\":\"%d\",\"z\":\"%d\"}},\n", received_packet.v1, received_packet.v2, received_packet.v3);
    }
  }

  leds_off(LEDS_RED);
}
static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
static struct broadcast_conn broadcast;
/*---------------------------------------------------------------------------*/
static void free_sink_busy_before(){
  if(!busy_node_start){
    sink_busy = false;
    busy_node_id = 0;
  }
}
/*---------------------------------------------------------------------------*/
static void free_sink_busy_after(){
  sink_busy = false;
  busy_node_id = 0;
}
// sensors
#if BOARD_SENSORTAG
/*---------------------------------------------------------------------------*/
/*
 Update sensor readings in a staggered fashion every SENSOR_READING_PERIOD,
 ACCEL_GYRO_SENSOR_READING_PERIOD, or MAGNTO_SENSOR_READING_PERIOD ticks
 */
static struct ctimer bmp_timer, opt_timer, tmp_timer, mpu_timer;
/*---------------------------------------------------------------------------*/
// set values when the sensor is active
static int pressure = -1;
static int light = -1;
static int temp = -1;
struct  accel {int x; int y; int z;};
static struct accel accel_data;
struct  gyro {int x; int y; int z;};
static struct gyro gyro_data;
struct  magnto {int x; int y; int z;};
static struct magnto magnto_data;
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
// static void
// print_mpu_reading(int reading)
// {
//   if(reading < 0) {
//     printf("-");
//     reading = -reading;
//   }

//   printf("%d.%02d", reading / 100, reading % 100);
// }
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
// to setup different sampling rates for different sensors
static int counter = 0;
static int start = 0;
static int accel_gyro_enabled = 0;
static int magnto_enabled = 0;
static int last_magnto_read = -2 * MAGNTO_SENSOR_READING_PERIOD;
/*---------------------------------------------------------------------------*/

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

  if(clock_seconds() - start < ACCEL_GYRO_MAGNTO_SAMPLING_DURATION){
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
// static void
// init_hdc_reading(void *not_used)
// {
//   SENSORS_ACTIVATE(hdc_1000_sensor);
// }
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
// static void
// get_sync_sensor_readings(void)
// {
//   int value;

//   printf("-----------------------------------------\n");

//   value = batmon_sensor.value(BATMON_SENSOR_TYPE_TEMP);
//   printf("Bat: Temp=%d C\n", value);

//   value = batmon_sensor.value(BATMON_SENSOR_TYPE_VOLT);
//   printf("Bat: Volt=%d mV\n", (value * 125) >> 5);

// #if BOARD_SMARTRF06EB
//   SENSORS_ACTIVATE(als_sensor);

//   value = als_sensor.value(0);
//   printf("ALS: %d raw\n", value);

//   SENSORS_DEACTIVATE(als_sensor);
// #endif

//   return;
// }
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
PROCESS_THREAD(cc26xx_data_collection_process, ev, data)
{
  static struct etimer et_test;
  PROCESS_EXITHANDLER(broadcast_close(&broadcast);)

  int check_fd;

  PROCESS_BEGIN();

  broadcast_open(&broadcast, 129, &broadcast_call);
  /* Init the BLE advertisement daemon */
  // rf_ble_beacond_config(0, BOARD_STRING);
  // rf_ble_beacond_start();

  if(node_id != SINK_ID){
    printf("CC26XX CFS data collection\n");

    leds_on(LEDS_ALL);

    check_fd = cfs_open(FILENAME, CFS_READ);
    if(check_fd != -1) {
      cfs_close(check_fd);
      printf("reserve file failed\n");

      /*check for file with size 1 K to sync to sink*/
      /*if not, don't send to sink but lose data and start over*/

      check_fd = cfs_open(CHECK_FILENAME, CFS_READ);
      if(check_fd != -1) {
        cfs_close(check_fd);
        cfs_remove(CHECK_FILENAME);
        
        /*read and send file to sink*/
        process_start(&sync_data, NULL);
        PROCESS_WAIT_EVENT();
        // sync_data_test();
        printf("done with reading file, will send done\n");

        /*remove file & reserve size
        make sure to remove old file and reserve space for the new file before start writing to it*/
        while(remove_reserve_file() == 0);
      }

      /*to get timestamp*/
      /*send to sink done*/
      data_packet.flag = 2;
      data_packet.src_id = node_id;
      data_packet.dst_id = SINK_ID;
      packetbuf_copyfrom(&data_packet, (int)sizeof(data_packet_struct));
      broadcast_send(&broadcast);

      /*wait for receiving timestamp*/
      /*wait for response from sink*/
      etimer_set(&et_test, 2 * CLOCK_SECOND);
      while(1){
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et_test));
        printf("check on flag\n");
        if(time_set == true)
          break;
        
        packetbuf_copyfrom(&data_packet, (int)sizeof(data_packet_struct));
        broadcast_send(&broadcast);
        printf("sent and waiting\n");

        etimer_reset(&et_test);
      }
      etimer_stop(&et_test);

      // /*turn off radio*/
      broadcast_close(&broadcast);
      NETSTACK_RADIO.off();

      // start writing again
      write_to_ext_flash = true;
      total_size = 0;
    } else{
      /*to get timestamp*/
      /*send to sink done*/
      data_packet.flag = 2;
      data_packet.src_id = node_id;
      data_packet.dst_id = SINK_ID;
      packetbuf_copyfrom(&data_packet, (int)sizeof(data_packet_struct));
      broadcast_send(&broadcast);

      /*wait for receiving timestamp*/
      /*wait for response from sink*/
      etimer_set(&et_test, 2 * CLOCK_SECOND);
      while(1){
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et_test));
        printf("check on flag\n");
        if(time_set == true)
          break;
        
        data_packet.flag = 2;
        data_packet.src_id = node_id;
        data_packet.dst_id = SINK_ID;
        packetbuf_copyfrom(&data_packet, (int)sizeof(data_packet_struct));
        broadcast_send(&broadcast);
        printf("sent and waiting\n");

        etimer_reset(&et_test);
      }
      etimer_stop(&et_test);

      /*turn off radio*/
      broadcast_close(&broadcast);
      NETSTACK_RADIO.off();

      /*start writing again*/
      write_to_ext_flash = true;
      total_size = 0;
    }

    leds_off(LEDS_ALL);

    init_sensors();

    etimer_set(&et, cc26xx_data_collection_loop_interval);

    init_sensor_readings();

    int fd;
    int n;

    while(1) {

      PROCESS_WAIT_EVENT();

      if(ev == PROCESS_EVENT_TIMER) {
        if(data == &et) {

          if(start_sync){
            watchdog_reboot();
          }

          // if(!write_to_ext_flash || total_size >= 1024){
          if(!write_to_ext_flash){
            /*send to sink asking to sync data*/      
            if(execute_once_flag){
              execute_once_flag = false;

              // write_to_ext_flash = false;

              // create a file with size 1 K as asign for clear reboot
              cfs_coffee_reserve(CHECK_FILENAME, CHECK_FILE_SIZE);

              /*stop the sensors*/
              SENSORS_DEACTIVATE(bmp_280_sensor);
              ctimer_stop(&bmp_timer);
              SENSORS_DEACTIVATE(tmp_007_sensor);
              ctimer_stop(&tmp_timer);
              ctimer_stop(&opt_timer);
              SENSORS_DEACTIVATE(mpu_9250_sensor);
              ctimer_stop(&mpu_timer);

              printf("will send ask to sync\n");

              /*turn on radio*/
              broadcast_open(&broadcast, 129, &broadcast_call);
              NETSTACK_RADIO.on();

              /*update time interval*/
              cc26xx_data_collection_loop_interval = 2 * CLOCK_SECOND;

              /*setup packet to send*/
              data_packet.flag = 1;
              data_packet.src_id = node_id;
              data_packet.dst_id = SINK_ID;
            }
            
            packetbuf_copyfrom(&data_packet, (int)sizeof(data_packet_struct));
            broadcast_send(&broadcast);
            printf("sent and waiting\n");
          }

          //
          if(pressure != -1 && temp != -1 && light != -1){
            // printf("%d: ", clock_seconds());
            // printf("Pressure = %d.%02d", pressure / 100, pressure % 100);
            // printf(", Temp = %d.%03d", temp / 1000, temp % 1000);
            // printf(", Light = %d.%02d\n", light / 100, light % 100);

            if(write_to_ext_flash && time_set){
              leds_on(LEDS_YELLOW);

              fd = cfs_open(FILENAME, CFS_WRITE | CFS_APPEND);

              if(fd != -1) {
                delimiter = SENSOR_TYPE.PRESSURE_TEMP_LIGHT;
                record_timestamp = timestamp + clock_seconds() - start_time_when_get_timestamp;
                v1 = pressure;
                v2 = temp;
                v3 = light;

                n = cfs_write(fd, &v1, (int)sizeof(v1));
                n += cfs_write(fd, &v2, (int)sizeof(v2));
                n += cfs_write(fd, &v3, (int)sizeof(v3));
                n += cfs_write(fd, &record_timestamp, (int)sizeof(record_timestamp));
                n += cfs_write(fd, &delimiter, (int)sizeof(delimiter));
                if(n != TOTAL_RECORD_SIZE) {
                  printf("failed to write %d bytes to %s, wrote %d\n", TOTAL_RECORD_SIZE, FILENAME, n);
                  cfs_close(fd);
                  write_to_ext_flash = false;
                  total_size = 0;
                } else{
                  total_size += TOTAL_RECORD_SIZE;
                  printf("total size = %d B = %d KB\t", total_size, (total_size / 1024));
                  printf("%d,%d,%d,%d,%d\n", delimiter, record_timestamp, v1, v2, v3);
                  cfs_close(fd);
                  // printf("successfully appended data to cfs. wrote %i bytes\n", n);
                }
              } else {
                printf("ERROR: could not write to memory.\n");
                write_to_ext_flash = false;
                total_size = 0;
                // leds_on(LEDS_RED);
              }
              leds_off(LEDS_YELLOW);
            }

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

            if(write_to_ext_flash && time_set){
              leds_on(LEDS_YELLOW);

              fd = cfs_open(FILENAME, CFS_WRITE | CFS_APPEND);

              if(fd != -1) {
                delimiter = SENSOR_TYPE.ACCEL;
                record_timestamp = timestamp + clock_seconds() - start_time_when_get_timestamp;
                v1 = accel_data.x;
                v2 = accel_data.y;
                v3 = accel_data.z;
                n = cfs_write(fd, &v1, (int)sizeof(v1));
                n += cfs_write(fd, &v2, (int)sizeof(v2));
                n += cfs_write(fd, &v3, (int)sizeof(v3));
                n += cfs_write(fd, &record_timestamp, (int)sizeof(record_timestamp));
                n += cfs_write(fd, &delimiter, (int)sizeof(delimiter));
                if(n != TOTAL_RECORD_SIZE) {
                  printf("failed to write %d bytes to %s\n", TOTAL_RECORD_SIZE, FILENAME);
                  cfs_close(fd);
                  write_to_ext_flash = false;
                  total_size = 0;
                } else{
                  total_size += TOTAL_RECORD_SIZE;
                  printf("total size = %d B = %d KB\t", total_size, (total_size / 1024));
                  printf("%d,%d,%d,%d,%d\n", delimiter, record_timestamp, v1, v2, v3);
                  // printf("successfully appended data to cfs. wrote %i bytes\n", n);

                  delimiter = SENSOR_TYPE.GYRO;
                  record_timestamp = timestamp + clock_seconds() - start_time_when_get_timestamp;
                  v1 = gyro_data.x;
                  v2 = gyro_data.y;
                  v3 = gyro_data.z;
                  n = cfs_write(fd, &v1, (int)sizeof(v1));
                  n += cfs_write(fd, &v2, (int)sizeof(v2));
                  n += cfs_write(fd, &v3, (int)sizeof(v3));
                  n += cfs_write(fd, &record_timestamp, (int)sizeof(record_timestamp));
                  n += cfs_write(fd, &delimiter, (int)sizeof(delimiter));
                  if(n != TOTAL_RECORD_SIZE) {
                    printf("failed to write %d bytes to %s\n", TOTAL_RECORD_SIZE, FILENAME);
                    cfs_close(fd);
                    write_to_ext_flash = false;
                    total_size = 0;
                  } else{
                    total_size += TOTAL_RECORD_SIZE;
                    printf("total size = %d B = %d KB\t", total_size, (total_size / 1024));
                    printf("%d,%d,%d,%d,%d\n", delimiter, record_timestamp, v1, v2, v3);
                    // printf("successfully appended data to cfs. wrote %i bytes\n", n);
                    cfs_close(fd);
                  }
                }
              } else {
                printf("ERROR: could not write to memory.\n");
                write_to_ext_flash = false;
                total_size = 0;
                // leds_on(LEDS_RED);
              }
              leds_off(LEDS_YELLOW);
            }

            if(magnto_enabled){
              // got reading with accel and gyro => print
              // printf("Magnto: x = ");
              // print_mpu_reading(magnto_data.x);
              // printf(", y = ");
              // print_mpu_reading(magnto_data.y);
              // printf(", z = ");
              // print_mpu_reading(magnto_data.z);
              // printf("\n");

              if(write_to_ext_flash && time_set){
                leds_on(LEDS_YELLOW);

                fd = cfs_open(FILENAME, CFS_WRITE | CFS_APPEND);

                if(fd != -1) {
                  delimiter = SENSOR_TYPE.MAGNTO;
                  record_timestamp = timestamp + clock_seconds() - start_time_when_get_timestamp;
                  v1 = magnto_data.x;
                  v2 = magnto_data.y;
                  v3 = magnto_data.z;

                  n = cfs_write(fd, &v1, (int)sizeof(v1));
                  n += cfs_write(fd, &v2, (int)sizeof(v2));
                  n += cfs_write(fd, &v3, (int)sizeof(v3));
                  n += cfs_write(fd, &record_timestamp, (int)sizeof(record_timestamp));
                  n += cfs_write(fd, &delimiter, (int)sizeof(delimiter));
                  if(n != TOTAL_RECORD_SIZE) {
                    printf("failed to write %d bytes to %s\n", TOTAL_RECORD_SIZE, FILENAME);
                    cfs_close(fd);
                    write_to_ext_flash = false;
                    total_size = 0;
                  } else{
                    total_size += TOTAL_RECORD_SIZE;
                    printf("total size = %d B = %d KB\t", total_size, (total_size / 1024));
                    printf("%d,%d,%d,%d,%d\n", delimiter, record_timestamp, v1, v2, v3);
                    cfs_close(fd);
                    // printf("successfully appended data to cfs. wrote %i bytes\n", n);
                  }
                } else {
                  printf("ERROR: could not write to memory.\n");
                  write_to_ext_flash = false;
                  total_size = 0;
                  // leds_on(LEDS_RED);
                }
                leds_off(LEDS_YELLOW);
              }
            }
          }
          etimer_set(&et, cc26xx_data_collection_loop_interval);
        }
      } else if(ev == PROCESS_EVENT_CONTINUE){
        watchdog_reboot();
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
                total_size = 0;
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
          // watchdog_periodic();
          get_mpu_reading();
  #elif BOARD_SMARTRF06EB
          printf("Sel: Pin %d, press duration %d clock ticks\n",
                 button_select_sensor.value(BUTTON_SENSOR_VALUE_STATE),
                 button_select_sensor.value(BUTTON_SENSOR_VALUE_DURATION));
  #endif
        }
      }
    }
  } else if(node_id == SINK_ID){
    // sink
    printf("CC26XX CFS data collection - sink\n");

    // for serial port
    #if !WITH_UIP && !WITH_UIP6
    uart1_set_input(serial_line_input_byte);
    serial_line_init();
    #endif

    while(1) {
      PROCESS_WAIT_EVENT();

      if (ev == serial_line_event_message) {
        leds_on(LEDS_RED);
        timestamp = atoi(data);
        if(timestamp != 0){
          time_set = true;
          start_time_when_get_timestamp = clock_seconds();
        }
        // printf("got input string: %s and %d\n", (char *) data, timestamp);
        leds_off(LEDS_RED);
      } else if(ev == PROCESS_EVENT_CONTINUE){
        // printf("got sync request, send response to start\n");
        timestamp = timestamp + clock_seconds() - start_time_when_get_timestamp;        
        // printf("time when node will start to sync after reboot = %d\n", timestamp);
        data_packet.flag = 1;
        data_packet.src_id = node_id;
        data_packet.dst_id = received_packet.src_id;
        data_packet.timestamp = timestamp;
        packetbuf_copyfrom(&data_packet, (int)sizeof(data_packet));
        broadcast_send(&broadcast);
      } else if(ev == PROCESS_EVENT_POLL){
        if(time_set){
          // printf("got done, send timestamp to %d\n", received_packet.src_id);
          data_packet.flag = 2;
          data_packet.src_id = node_id;
          data_packet.dst_id = received_packet.src_id;
          timestamp = timestamp + clock_seconds() - start_time_when_get_timestamp;
          // printf("time when node stop = %d\n", timestamp);
          data_packet.timestamp = timestamp;
          packetbuf_copyfrom(&data_packet, (int)sizeof(data_packet));
          broadcast_send(&broadcast);
        }
      }
    }
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
// process
PROCESS_THREAD(sync_data, ev, data){
  int r, fd;
  static struct etimer et_test;

  PROCESS_BEGIN();

  printf("\n\n\n***START READING***\n\n\n");

  // read cfs file
  fd = cfs_open(FILENAME, CFS_READ);
  if(fd != -1) {
    printf("file opened\n");
    if(cfs_seek(fd, 0, CFS_SEEK_SET) != 0) {
      printf("seek failed\n");
      cfs_close(fd);
    } else{
      for(;;) {
        etimer_set(&et_test, CLOCK_SECOND/100);
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et_test));

        r = cfs_read(fd, &v1, sizeof(v1));
        r += cfs_read(fd, &v2, sizeof(v2));
        r += cfs_read(fd, &v3, sizeof(v3));
        r += cfs_read(fd, &record_timestamp, sizeof(record_timestamp));
        r += cfs_read(fd, &delimiter, sizeof(delimiter));

        if(r == 0) {
          break;
        } else if(r < TOTAL_RECORD_SIZE) {
          printf("failed to read %d bytes from %s, got %d\n", TOTAL_RECORD_SIZE, FILENAME, r);
          break;
        }

        // printf("%d,%d,%d,%d,%d\n", delimiter, record_timestamp, v1, v2, v3);

        // send data to sink
        data_packet.flag = 3;
        data_packet.src_id = node_id;
        data_packet.dst_id = SINK_ID;
        data_packet.timestamp = record_timestamp;
        data_packet.v1 = v1;
        data_packet.v2 = v2;
        data_packet.v3 = v3;
        data_packet.delimiter = delimiter;
        packetbuf_copyfrom(&data_packet, (int)sizeof(data_packet));
        broadcast_send(&broadcast);
      }
      etimer_stop(&et_test);
      cfs_close(fd);
      printf("\n\n\n***END READING with timestamp %d***\n\n\n", record_timestamp);
     }
  } else {
    printf("failed to open %s\n", FILENAME);
  }
  process_post(&cc26xx_data_collection_process, PROCESS_EVENT_CONTINUE, NULL);
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
static int remove_reserve_file() {
  // remove and reserve file
  cfs_remove(FILENAME);
  if(cfs_coffee_reserve(FILENAME, FILE_SIZE) < 0) {
    printf("reserve file failed\n");
    return 0;
  } else{
    return 1;
  }
}
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 * @}
 */