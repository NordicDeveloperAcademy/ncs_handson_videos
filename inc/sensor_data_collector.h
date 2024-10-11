#ifndef SENSOR_DATA_COLLECTOR_H
#define SENSOR_DATA_COLLECTOR_H
#include <zephyr/drivers/sensor.h>

struct k_msgq *get_sensor_readings_queue(void);

typedef struct {
	struct sensor_value temp;     // BME688
	struct sensor_value press;    // BME688
	struct sensor_value humidity; // BME688
#ifndef CONFIG_SIMULATED_SENSOR
	struct sensor_value gas_res; // BME688
	struct sensor_value gyr[3];  // BMI270
#endif
	struct sensor_value acc[3]; // BMI270
} sensorsreadings;

#ifndef CONFIG_SIMULATED_SENSOR
#define ENV_VALUES_CNT 4
#else
#define ENV_VALUES_CNT 3
#endif

#endif