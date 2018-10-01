/*	Author: ebramkw
	Typedef and definitions	*/

/*---------------------------------------------------------------------------*/
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
// Sample temp, pressure, and light every 2 seconds
#define SENSOR_READING_PERIOD (CLOCK_SECOND * 2)
// whenever sample, sample with 20 HZ
#define ACCEL_GYRO_MAGNTO_SAMPLING_PERIOD (CLOCK_SECOND * 0.05)
// Sample every minute
#define ACCEL_GYRO_SENSOR_READING_PERIOD (CLOCK_SECOND * 60)
// Sample every 30 minutes
#define MAGNTO_SENSOR_READING_PERIOD (60 * 30)
// sample for this duration, in seconds, every period with sampling rate
#define ACCEL_GYRO_MAGNTO_SAMPLING_DURATION 5
/*---------------------------------------------------------------------------*/
// file to store data and its size
#define FILENAME "data"
#define FILE_SIZE 500 * 1024
// check file, to handle unexcpected reboot
#define CHECK_FILENAME "check"
#define CHECK_FILE_SIZE 1024
/*---------------------------------------------------------------------------*/
#define SINK_ID 18051
// mapping node_id to object
#define DATA_COLLECTION_NODES 9
#define NODE_ID_MAPPING     {49280, 10113, 10626, 27393, 32129, 37379, 63620, 49924, 53893}
#define NODE_OBJECT_MAPPING {"door_outdoor", "chair-1", "chair_2", "window_indoor", "window_outdoor", "desk", "door_indoor", "screen", "person"}
/*---------------------------------------------------------------------------*/
#define FREE_SINK_NOT_STARTED_DURATION 60 * CLOCK_SECOND
#define FREE_SINK_NOT_FINISHED_DURATION 15 * 60 * CLOCK_SECOND
/*---------------------------------------------------------------------------*/
typedef struct {
  uint8_t flag;
  unsigned short src_id;
  unsigned short dst_id;
  int timestamp;

  int v1;
  int16_t v2;
  int16_t v3;
  uint8_t delimiter;

} data_packet_struct;
/*---------------------------------------------------------------------------*/
typedef struct{
  uint8_t PRESSURE_TEMP_LIGHT;
  uint8_t ACCEL;
  uint8_t GYRO;
  uint8_t MAGNTO;
} SENSOR_TYPE_STRUCT;
SENSOR_TYPE_STRUCT x = {1, 2, 3, 4};
#define SENSOR_TYPE x
/*---------------------------------------------------------------------------*/
/*	record
	defined as single variables instead of struct to make the size of a single record equals to 13 Bytes
	if defined as a struct, the size will be multiple of 4 = 16 Bytes
	record size = 13 bytes	*/
int v1;
int16_t v2;
int16_t v3;
int record_timestamp;
uint8_t delimiter;
#define TOTAL_RECORD_SIZE (int)(sizeof(v1) + sizeof(v2) + sizeof(v3) + sizeof(record_timestamp) + sizeof(delimiter))
/*---------------------------------------------------------------------------*/