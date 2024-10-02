#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>

#include "sensor_data_collector.h"

#define SENSOR_THREAD_PRIORITY	7
#define SENSOR_THREAD_STACKSIZE 1024

#ifdef CONFIG_SIMULATED_SENSOR
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
#else
static const struct device *get_bme688_sensor(void)
{
	const struct device *const dev = DEVICE_DT_GET(DT_NODELABEL(bme688));
	;

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

static const struct device *get_bmi270_sensor(void)
{
	const struct device *const dev = DEVICE_DT_GET(DT_NODELABEL(bmi270));
	struct sensor_value full_scale, sampling_freq, oversampling;

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
	/* Setting scale in G, due to loss of precision if the SI unit m/s^2
	 * is used
	 */
	full_scale.val1 = 2; /* G */
	full_scale.val2 = 0;
	sampling_freq.val1 = 100; /* Hz. Performance mode */
	sampling_freq.val2 = 0;
	oversampling.val1 = 1; /* Normal mode */
	oversampling.val2 = 0;

	sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, &full_scale);
	sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_OVERSAMPLING, &oversampling);
	/* Set sampling frequency last as this also sets the appropriate
	 * power mode. If already sampling, change to 0.0Hz before changing
	 * other attributes
	 */
	sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &sampling_freq);

	/* Setting scale in degrees/s to match the sensor scale */
	full_scale.val1 = 500; /* dps */
	full_scale.val2 = 0;
	sampling_freq.val1 = 100; /* Hz. Performance mode */
	sampling_freq.val2 = 0;
	oversampling.val1 = 1; /* Normal mode */
	oversampling.val2 = 0;

	sensor_attr_set(dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_FULL_SCALE, &full_scale);
	sensor_attr_set(dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_OVERSAMPLING, &oversampling);
	/* Set sampling frequency last as this also sets the appropriate
	 * power mode. If already sampling, change sampling frequency to
	 * 0.0Hz before changing other attributes
	 */
	sensor_attr_set(dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &sampling_freq);
	printk("Found device \"%s\", getting sensor data\n", dev->name);
	return dev;
}

#endif

int sensor_data_collector(void)
{
#ifdef CONFIG_SIMULATED_SENSOR    
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

#else
	const struct device *dev_bme688 = get_bme688_sensor();
	const struct device *dev_bmi270 = get_bmi270_sensor();
	if (dev_bme688 == NULL) {
		return 0;
	}
	if (dev_bmi270 == NULL) {
		return 0;
	}
	while (1) {
		sensorsreadings value;
		sensor_sample_fetch(dev_bmi270);
		sensor_sample_fetch(dev_bme688);

		sensor_channel_get(dev_bme688, SENSOR_CHAN_AMBIENT_TEMP, &value.temp);
		sensor_channel_get(dev_bme688, SENSOR_CHAN_PRESS, &value.press);
		sensor_channel_get(dev_bme688, SENSOR_CHAN_HUMIDITY, &value.humidity);
		sensor_channel_get(dev_bme688, SENSOR_CHAN_GAS_RES, &value.gas_res);
		sensor_channel_get(dev_bmi270, SENSOR_CHAN_ACCEL_XYZ, value.acc);
		sensor_channel_get(dev_bmi270, SENSOR_CHAN_GYRO_XYZ, value.gyr);

		printk("Sensor Thread Reporting!\n");
		printk("AX: %d.%06d; AY: %d.%06d; AZ: %d.%06d; "
		       "GX: %d.%06d; GY: %d.%06d; GZ: %d.%06d;\n",
		       value.acc[0].val1, value.acc[0].val2, value.acc[1].val1, value.acc[1].val2,
		       value.acc[2].val1, value.acc[2].val2, value.gyr[0].val1, value.gyr[0].val2,
		       value.gyr[1].val1, value.gyr[1].val2, value.gyr[2].val1, value.gyr[2].val2);
		printk("T: %d.%06d; P: %d.%06d; H: %d.%06d; G: %d.%06d\n", value.temp.val1,
		       value.temp.val2, value.press.val1, value.press.val2, value.humidity.val1,
		       value.humidity.val2, value.gas_res.val1, value.gas_res.val2);
		k_sleep(K_SECONDS(CONFIG_APP_CONTROL_SAMPLING_INTERVAL_S));
	}

#endif
}

K_THREAD_DEFINE(sensor_data_collector_id, SENSOR_THREAD_STACKSIZE, sensor_data_collector, NULL,
		NULL, NULL, SENSOR_THREAD_PRIORITY, 0, 1000);