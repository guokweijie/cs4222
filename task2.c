#include <stdio.h>

#include "contiki.h"
#include "sys/etimer.h"
#include "sys/rtimer.h"
#include "buzzer.h"

#include "board-peripherals.h"

#include <stdint.h>
#include <math.h>

#define LIGHT_THRESHOLD 300
#define GYRO_THRESHOLD 5000
#define ACC_THRESHOLD 30
#define BUZZER_FREQUENCY 2093

static int counter_etimer;
static int counter_rtimer;
static struct etimer wait_etimer, cycle_etimer;
static struct rtimer buzz_rtimer;

static int prev_light_reading;
static int prev_gyro_x, prev_gyro_y, prev_gyro_z;
static int prev_acc_x, prev_acc_y, prev_acc_z;

enum state
{
  STATE_IDLE,
  STATE_BUZZ,
  STATE_WAIT
} current_state = STATE_IDLE;

static void init_opt_reading(void);
static int get_light_reading(void);
static void init_mpu_reading(void);

PROCESS(task2, "Task 2");
AUTOSTART_PROCESSES(&task2);

void buzzer_callback(struct rtimer *timer, void *ptr)
{
  current_state = STATE_WAIT;
  printf("State: WAIT\n\n");
  buzzer_stop();
  etimer_set(&wait_etimer, CLOCK_SECOND * 2);
  printf("Waiting for 2 seconds\n\n");
  process_poll(&task2);
}

static int
get_light_reading()
{
  int value;

  value = opt_3001_sensor.value(0);
  if (value != CC26XX_SENSOR_READING_ERROR)
  {
    printf("OPT: Light=%d.%02d lux\n", value / 100, value % 100);
  }
  else
  {
    printf("OPT: Light Sensor's Warming Up\n\n");
    value = 0;
  }
  init_opt_reading();
  return value / 100;
}

static void
init_opt_reading(void)
{
  SENSORS_ACTIVATE(opt_3001_sensor);
}

static bool
is_change_significant()
{
  bool significant = false;
  int light_reading = get_light_reading();
  int gyro_x = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_X);
  int gyro_y = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Y);
  int gyro_z = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Z);
  int acc_x = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_X);
  int acc_y = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Y);
  int acc_z = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Z);
  printf("mpu readings: %d, %d, %d, %d, %d, %d\n", gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z);
  int acc_mag = sqrt(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z);

  if (abs(light_reading - prev_light_reading) > LIGHT_THRESHOLD)
  {
    printf("Light reading changed from %d to %d\n", prev_light_reading, light_reading);
    significant = true;
  }

  if (abs(gyro_x) > GYRO_THRESHOLD || abs(gyro_y) > GYRO_THRESHOLD || abs(gyro_z) > GYRO_THRESHOLD)
  {
    printf("Gyro reading changed from %d, %d, %d to %d, %d, %d\n", prev_gyro_x, prev_gyro_y, prev_gyro_z, gyro_x, gyro_y, gyro_z);
    significant = true;
  }

  if (abs(acc_mag - 100) > ACC_THRESHOLD)
  {
    printf("Acc mag: %d, abs(acc_mag - 1): %d\n", acc_mag, abs(acc_mag - 1));
    printf("Acc reading changed from %d, %d, %d to %d, %d, %d\n", prev_acc_x, prev_acc_y, prev_acc_z, acc_x, acc_y, acc_z);
    significant = true;
  }

  prev_light_reading = light_reading;
  prev_gyro_x = gyro_x;
  prev_gyro_y = gyro_y;
  prev_gyro_z = gyro_z;
  prev_acc_x = acc_x;
  prev_acc_y = acc_y;
  prev_acc_z = acc_z;

  return significant;
}

static void
init_mpu_reading(void)
{
  mpu_9250_sensor.configure(SENSORS_ACTIVE, MPU_9250_SENSOR_TYPE_ALL);
}

PROCESS_THREAD(task2, ev, data)
{
  static struct etimer sensor_etimer;
  PROCESS_BEGIN();
  buzzer_init();
  init_mpu_reading();
  init_opt_reading();
  prev_light_reading = get_light_reading();
  etimer_set(&sensor_etimer, CLOCK_SECOND / 4);
  while (1)
  {
    if (current_state == STATE_IDLE)
    {
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&sensor_etimer));
      if (is_change_significant())
      {
        printf("Significant change detected\n\n");
        current_state = STATE_BUZZ;
        printf("State: BUZZ\n\n");
        buzzer_start(BUZZER_FREQUENCY);
        rtimer_set(&buzz_rtimer, RTIMER_NOW() + RTIMER_SECOND * 2, 0, buzzer_callback, NULL);
        etimer_set(&cycle_etimer, CLOCK_SECOND * 16);
        PROCESS_YIELD();
      } else {
        etimer_reset(&sensor_etimer);
        counter_etimer++;
        printf("Time(E): %d (cnt) \n", counter_etimer);
      }
    }
    else
    {
      printf("Waiting for buzzer to stop\n\n");
      PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_POLL);
      printf("polled\n\n");
      etimer_set(&wait_etimer, CLOCK_SECOND * 2);
      PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);
      if (data == &cycle_etimer)
      {
        current_state = STATE_IDLE;
        printf("State: IDLE\n\n");
      }
      else
      {
        current_state = STATE_BUZZ;
        printf("State: BUZZ\n\n");
        buzzer_start(BUZZER_FREQUENCY);
        rtimer_set(&buzz_rtimer, RTIMER_NOW() + RTIMER_SECOND * 2, 0, buzzer_callback, NULL);
      }
    }
  }
  PROCESS_END();
}