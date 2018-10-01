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
#include "dev/serial-line.h"
#include "dev/uart1.h"
#include "node-id.h"
#include "net/netstack.h"

/*---------------------------------------------------------------------------*/
static void init_bmp_reading(void *not_used);
static void init_opt_reading(void *not_used);
static void init_tmp_reading(void *not_used);
static void init_mpu_reading(void *not_used);
static int remove_reserve_file();
static void free_sink_busy_before();
static void free_sink_busy_after();
static char* get_mapping_object(uint16_t);
/*---------------------------------------------------------------------------*/