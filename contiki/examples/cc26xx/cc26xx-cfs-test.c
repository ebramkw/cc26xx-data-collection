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

#include <string.h>
#include "cfs/cfs.h"
#include "cfs/cfs-coffee.h"

PROCESS(cc26xx_demo_process, "cc26xx demo process");
AUTOSTART_PROCESSES(&cc26xx_demo_process);
/*---------------------------------------------------------------------------*/
#define FILENAME "data"
// #define FILENAME2 "data2"

#define FILE_SIZE 511 * 1024

int total_size = 0;

struct record {
  int sensor_data[3];
  int timestamp;
  uint8_t delimiter;
  // uint8_t end;
} record;

/*---------------------------------------------------------------------------*/
static int
dir_test(void)
{
  struct cfs_dir dir;
  struct cfs_dirent dirent;

  /* Coffee provides a root directory only. */
  if(cfs_opendir(&dir, "/") != 0) {
    printf("failed to open the root directory\n");
    return 0;
  }

  /* List all files and their file sizes. */
  printf("Available files\n");
  while(cfs_readdir(&dir, &dirent) == 0) {
    printf("%s (%lu bytes)\n", dirent.name, (unsigned long)dirent.size);
  }

  cfs_closedir(&dir);

  return 1;
}
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(cc26xx_demo_process, ev, data)
{
  PROCESS_BEGIN();

  leds_on(LEDS_ALL);

  cfs_remove(FILENAME);
  // cfs_remove(FILENAME2);
  // cfs_coffee_format();

  if(cfs_coffee_reserve(FILENAME, FILE_SIZE) < 0) {
    printf("reserve %s file failed\n", FILENAME);
  }

  // if(cfs_coffee_reserve(FILENAME2, FILE_SIZE) < 0) {
  //   printf("reserve %s file failed\n", FILENAME2);
  // }

  leds_off(LEDS_ALL);

  printf("CC26XX CFS data collection\n");

  int fd;
  int n, r;

  record.delimiter = 4;
  record.timestamp = clock_seconds();
  record.sensor_data[0] = 1000;
  record.sensor_data[1] = 2000;
  record.sensor_data[2] = 3000;

  leds_on(LEDS_YELLOW);
  total_size = 0;
  while(1){
    fd = cfs_open(FILENAME, CFS_WRITE | CFS_APPEND);

    if(fd != -1) {
      n = cfs_write(fd, &record, (int)sizeof(record));
      if(n != sizeof(record)) {
        printf("failed to write %d bytes to %s\n", (int)sizeof(record), FILENAME);
        cfs_close(fd);
        break;
      } else{
        total_size += (int)sizeof(record);
        // printf("total_size = %d\t", total_size);
        // printf("%d,%d,%d,%d,%d\n", record.delimiter, record.timestamp, record.sensor_data[0], record.sensor_data[1], record.sensor_data[2]);
        cfs_close(fd);
        // printf("successfully appended data to cfs. wrote %i bytes\n", n);
      }
    } else {
      printf("ERROR: could not write to memory.\n");
      break;
      leds_on(LEDS_RED);
    }
  }
  printf("total_size = %d B = %d KB\n", total_size, (total_size/(1024)));

  // total_size = 0;
  // while(1){
  //   fd = cfs_open(FILENAME2, CFS_WRITE | CFS_APPEND);

  //   if(fd != -1) {
  //     n = cfs_write(fd, &record, (int)sizeof(record));
  //     if(n != sizeof(record)) {
  //       printf("failed to write %d bytes to %s\n", (int)sizeof(record), FILENAME2);
  //       cfs_close(fd);
  //       break;
  //     } else{
  //       total_size += (int)sizeof(record);
  //       // printf("total_size = %d\t", total_size);
  //       // printf("%d,%d,%d,%d,%d\n", record.delimiter, record.timestamp, record.sensor_data[0], record.sensor_data[1], record.sensor_data[2]);
  //       cfs_close(fd);
  //       // printf("successfully appended data to cfs. wrote %i bytes\n", n);
  //     }
  //   } else {
  //     printf("ERROR: could not write to memory.\n");
  //     break;
  //   }
  // }
  // printf("total_size = %d B = %d KB\n", total_size, (total_size/(1024)));
  
  leds_off(LEDS_YELLOW);


  if(dir_test() == 0) {
    printf("dir test failed\n");
  }

  printf("***START READING %s***\n", FILENAME);

  // read cfs file
  fd = cfs_open(FILENAME, CFS_READ);
  if(fd != -1) {
    if(cfs_seek(fd, 0, CFS_SEEK_SET) != 0) {
      printf("seek failed\n");
      cfs_close(fd);
    } else{
      leds_on(LEDS_ALL);
      for(;;) {
        r = cfs_read(fd, &record, sizeof(record));

        if(r == 0) {
          break;
        } else if(r < sizeof(record)) {
          printf("failed to read %d bytes from %s, got %d\n", (int)sizeof(record), FILENAME, r);
          break;
        }

        // printf("%d,%d,%d,%d,%d\n", record.delimiter, record.timestamp, record.sensor_data[0], record.sensor_data[1], record.sensor_data[2]);
      }
      cfs_close(fd);
      printf("***END READING***\n");

      leds_off(LEDS_ALL);
    }
  } else{
    printf("failed to open %s\n", FILENAME);
  }

  // printf("***START READING %s***\n", FILENAME2);

  // // read cfs file
  // fd = cfs_open(FILENAME2, CFS_READ);
  // if(fd != -1) {
  //   if(cfs_seek(fd, 0, CFS_SEEK_SET) != 0) {
  //     printf("seek failed\n");
  //     cfs_close(fd);
  //   } else{
  //     leds_on(LEDS_ALL);
  //     for(;;) {
  //       r = cfs_read(fd, &record, sizeof(record));

  //       if(r == 0) {
  //         break;
  //       } else if(r < sizeof(record)) {
  //         printf("failed to read %d bytes from %s, got %d\n", (int)sizeof(record), FILENAME, r);
  //         break;
  //       }

  //       // printf("%d,%d,%d,%d,%d\n", record.delimiter, record.timestamp, record.sensor_data[0], record.sensor_data[1], record.sensor_data[2]);
  //     }
  //     cfs_close(fd);
  //     printf("***END READING***\n");

  //     leds_off(LEDS_ALL);
  //   }
  // } else{
  //   printf("failed to open %s\n", FILENAME2);
  // }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 * @}
 */
