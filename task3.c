/*
IDLE -> INTERIM: Significant change detected in motion
INTERIM -> BUZZ: Significant change detected in light (BUZZ 2 seconds, WAIT 4 seconds)
BUZZ -> IDLE: Significant change detected in light
*/

#define LIGHT_THRESHOLD 30000 // in centi-lux
#define GYRO_THRESHOLD 5000
#define ACC_THRESHOLD 30
#define ERROR_CODE -1
#define NO_READING INT32_MIN

#define POLL_FREQUENCY_MILLIS 250

#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "contiki.h"
#include "sys/etimer.h"
#include "sys/rtimer.h"
#include "buzzer.h"
#include "board-peripherals.h"

enum state {
    STATE_IDLE,
    STATE_INTERIM,
    STATE_BUZZ,
    STATE_WAIT
} current_state = IDLE;

struct gyro_acc_reading {
    int gyro_x;
    int gyro_y;
    int gyro_z;
    int acc_x;
    int acc_y;
    int acc_z;
    int mag;
};

static void init_sensors(void);
static int get_light_reading(void);
static bool is_light_change_significant(int);
static struct gyro_acc_reading get_gyro_acc_reading(void);
static bool is_gyro_acc_change_significant(struct gyro_acc_reading);

// Initialize prev gyro and acc reading to NO_READING
static struct gyro_acc_reading prev_reading = {NO_READING, NO_READING, NO_READING, NO_READING, NO_READING, NO_READING, NO_READING};

// Initialize prev light reading to -1
static int prev_light_reading = NO_READING;

PROCESS(task3, "Task 3");
AUTOSTART_PROCESSES(&task3);

static void init_sensors() {

    mpu_9250_sensor.configure(SENSORS_ACTIVE, MPU_9250_SENSOR_TYPE_ALL);
    SENSORS_ACTIVATE(opt_3001_sensor);

}

static int get_light_reading() {

    int value = opt_3001_sensor.value(0);

    if (value == CC26XX_SENSOR_READING_ERROR) {
        return ERROR_CODE;
    }

    // It's unclear why this is necessary, but it is in the example code
    SENSORS_ACTIVATE(opt_3001_sensor);

    return value;
}

static bool is_light_change_significant(int light_reading) {

    if (light_reading == ERROR_CODE) {
        return false;
    } else if (prev_light_reading == NO_READING) {
        prev_light_reading = light_reading;
        return false;
    }

    bool significant = false;

    if (abs(light_reading - prev_light_reading) > LIGHT_THRESHOLD)
    {
        printf("Light reading changed from %d to %d\n", prev_light_reading, light_reading);
        significant = true;
    }

    prev_light_reading = light_reading;
    return significant;

}

static struct gyro_acc_reading get_gyro_acc_reading() {

    struct gyro_acc_reading reading;

    reading.gyro_x = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_X);
    reading.gyro_y = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Y);
    reading.gyro_z = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Z);
    reading.acc_x = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_X);
    reading.acc_y = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Y);
    reading.acc_z = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Z);
    reading.mag = sqrt(reading.acc_x * reading.acc_x + reading.acc_y * reading.acc_y + reading.acc_z * reading.acc_z);

    return reading;

}

static bool is_gyro_acc_change_significant(struct gyro_acc_reading reading) {

    if (prev_reading.gyro_x == NO_READING) {
        prev_reading = reading;
        return false;
    }

    bool significant = false;

    if (abs(reading.gyro_x) > GYRO_THRESHOLD || abs(reading.gyro_y) > GYRO_THRESHOLD || abs(reading.gyro_z) > GYRO_THRESHOLD)
    {
        printf("Gyro reading changed from %d, %d, %d to %d, %d, %d\n", prev_reading.gyro_x, prev_reading.gyro_y, prev_reading.gyro_z, reading.gyro_x, reading.gyro_y, reading.gyro_z);
        significant = true;
    }

    if (abs(reading.mag - 100) > ACC_THRESHOLD)
    {
        printf("Acc mag: %d, abs(acc_mag - 1): %d\n", reading.mag, abs(reading.mag - 1));
        printf("Acc reading changed from %d, %d, %d to %d, %d, %d\n", prev_reading.acc_x, prev_reading.acc_y, prev_reading.acc_z, reading.acc_x, reading.acc_y, reading.acc_z);
        significant = true;
    }

    prev_reading = reading;
    return significant;

}

bool first_buzz_iter = true;
bool first_wait_iter = true;
clock_time_t start_time_buzz;
clock_time_t start_time_wait;

PROCESS_THREAD(task3, ev, data) {

    PROCESS_BEGIN();

    buzzer_init();
    init_sensors();

    static struct etimer etimer_poll;

    printf("Task 3 started\n");

    while (1) {

        switch (current_state) {
            case STATE_IDLE: {
                if (is_gyro_acc_change_significant(get_gyro_acc_reading())) {
                    current_state = STATE_INTERIM;
                    printf("Entering INTERIM state\n");
                }
                break;
            }
            case STATE_INTERIM: {
                if (is_light_change_significant(get_light_reading())) {
                    current_state = STATE_BUZZ;
                    printf("Entering BUZZ state from INTERIM\n");
                }
                break;
            }
            case STATE_BUZZ: { // nonblocking buzz

                printf("First buzz iteration %d \n", first_buzz_iter);
                
                if (first_buzz_iter) {
                    start_time_buzz = clock_time();
                    first_buzz_iter = false;
                    buzzer_start(2093);
                    printf("First buzz iteration (2) %d \n", first_buzz_iter);
                }

                printf("Time difference: %d %d\n", clock_time() - start_time_buzz, 2 * CLOCK_SECOND);

                if (clock_time() - start_time_buzz >= 2 * CLOCK_SECOND) {
                    current_state = STATE_WAIT;
                    first_buzz_iter = true;
                    printf("First buzz iteration (3) %d \n", first_buzz_iter);
                    buzzer_stop();
                    printf("Entering WAIT state\n");
                } else if (is_light_change_significant(get_light_reading())) {
                    current_state = STATE_IDLE;
                    first_buzz_iter = true;
                    printf("First buzz iteration (3) %d \n", first_buzz_iter);
                    buzzer_stop();
                    printf("Entering IDLE state\n");
                }

                break;

            }
            case STATE_WAIT: { // nonblocking wait

                if (first_wait_iter) {
                    first_wait_iter = false;
                    start_time_wait = clock_time();
                }

                if (clock_time() - start_time_wait >= 4 * CLOCK_SECOND) {
                    current_state = STATE_BUZZ;
                    first_wait_iter = true;
                    printf("Entering BUZZ state from WAIT\n");
                }

                break;
            }
        }

        etimer_set(&etimer_poll, POLL_FREQUENCY_MILLIS * CLOCK_SECOND / 1000);
        PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);

    }

    PROCESS_END();
}
