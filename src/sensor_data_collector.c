#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>

#include "sensor_data_collector.h"

#define SENSOR_THREAD_PRIORITY	7
#define SENSOR_THREAD_STACKSIZE 1024

static const struct device *get_simulated_sensor(void)
{
	const struct device *const dev = DEVICE_DT_GET(DT_NODELABEL(sensor_sim));

	if (dev == NULL) {
		/* No such node, or the node does not have status "okay". */
		printk("\nError: no device found.\n");
		return NULL;
	}

	if (!device_is_ready(dev)) {
		printk("\nError: Device \"%s\" is not ready; "
		       "check the driver initialization logs for errors.\n",
		       dev->name);
		return NULL;
	}

	printk("Found device \"%s\", getting sensor data\n", dev->name);
	return dev;
}

int sensor_data_collector(void)
{
	const struct device *sim_dev = get_simulated_sensor();

	if (sim_dev == NULL) {
		return 0;
	}

	while (1) {
		sensorsreadings value;
		sensor_sample_fetch(sim_dev);
		// Simulated sensor
		sensor_channel_get(sim_dev, SENSOR_CHAN_AMBIENT_TEMP, &value.temp);
		sensor_channel_get(sim_dev, SENSOR_CHAN_PRESS, &value.press);
		sensor_channel_get(sim_dev, SENSOR_CHAN_HUMIDITY, &value.humidity);
		sensor_channel_get(sim_dev, SENSOR_CHAN_ACCEL_XYZ, value.acc);

		printk("Sensor Thread Reporting!\n");
		printk("AX: %d.%06d; AY: %d.%06d; AZ: %d.%06d\n", value.acc[0].val1,
		       value.acc[0].val2, value.acc[1].val1, value.acc[1].val2, value.acc[2].val1,
		       value.acc[2].val2);
		printk("T: %d.%06d; P: %d.%06d; H: %d.%06d\n", value.temp.val1, value.temp.val2,
		       value.press.val1, value.press.val2, value.humidity.val1,
		       value.humidity.val2);
		k_sleep(K_SECONDS(CONFIG_APP_CONTROL_SAMPLING_INTERVAL_S));
	}
}

K_THREAD_DEFINE(sensor_data_collector_id, SENSOR_THREAD_STACKSIZE, sensor_data_collector, NULL,
		NULL, NULL, SENSOR_THREAD_PRIORITY, 0, 1000);