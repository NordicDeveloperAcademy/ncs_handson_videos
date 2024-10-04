#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/bluetooth/conn.h>
#include <bluetooth/services/nsms.h>
#include "sensor_data_collector.h"

#define SENSOR_THREAD_PRIORITY	7
#define SENSOR_THREAD_STACKSIZE 1024
#define BT_UUID_CSERVICE_VAL	BT_UUID_128_ENCODE(0xc2768000, 0xc410, 0x47e5, 0xbda9, 0x97fa0b0b9a99)
#define BT_UUID_INTERVAL_VAL	BT_UUID_128_ENCODE(0xc2768001, 0x1212, 0xefde, 0x1523, 0x785feabcd123)
#define BT_UUID_C_SERVICE	BT_UUID_DECLARE_128(BT_UUID_CSERVICE_VAL)
#define BT_UUID_S_INTERVAL	BT_UUID_DECLARE_128(BT_UUID_INTERVAL_VAL)
#define BUF_SIZE		64

BT_NSMS_DEF(nsms_imu, "IMU", false, "Unknown", BUF_SIZE);
BT_NSMS_DEF(nsms_env, "Environmental", false, "Unknown", BUF_SIZE);
static struct bt_conn *current_conn = NULL;
static int16_t sampling_interval = CONFIG_APP_CONTROL_SAMPLING_INTERVAL_S;
extern const k_tid_t sensor_data_collector_id;
static ssize_t sampling_interval_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				      void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &sampling_interval,
				 sizeof(sampling_interval));
}

static ssize_t sampling_interval_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				       const void *buf, uint16_t len, uint16_t offset,
				       uint8_t flags)
{
	uint16_t new_interval;
	if (len != sizeof(sampling_interval)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	memcpy(&new_interval, buf, sizeof(new_interval));

	if (new_interval < 10 || new_interval > 5400) {
		printk("Sampling interval out of allowed range: %d\n", new_interval);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	sampling_interval = new_interval;
	k_wakeup(sensor_data_collector_id);
	printk("New sampling interval %d seconds\n", new_interval);
	return len;
}

BT_GATT_SERVICE_DEFINE(
	custom_interval_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_C_SERVICE),
	BT_GATT_CHARACTERISTIC(BT_UUID_S_INTERVAL, // Sampling Interval Characteristic
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, sampling_interval_read,
			       sampling_interval_write, &sampling_interval)

);
static void connected(struct bt_conn *conn, uint8_t err)
{
	current_conn = bt_conn_ref(conn);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
}

BT_CONN_CB_DEFINE(conn_callbacks1) = {
	.connected = connected,
	.disconnected = disconnected,

};
static bool send_sensor_value(const struct sensor_value *val, size_t size, const char *channel)
{
	float float_data[size];
	char buf[BUF_SIZE];

	int len = sprintf(buf, "%s ", channel);

	for (size_t i = 0; i < size; i++) {
		float_data[i] = sensor_value_to_float(&val[i]);

		len += sprintf(buf + len, "%.6f ", (double)float_data[i]);
	}

	if (!strcmp(channel, "gyr")) {
		bt_nsms_set_status(&nsms_imu, buf);
	} else if (!strcmp(channel, "acc")) {
		bt_nsms_set_status(&nsms_imu, buf);
	} else {
		bt_nsms_set_status(&nsms_env, buf);
	}

	return true;
}

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
		if (current_conn != NULL) {
			send_sensor_value(&value.temp, 1, "temp");
			send_sensor_value(&value.press, 1, "press");
			send_sensor_value(&value.humidity, 1, "humidity");
			send_sensor_value(value.acc, 3, "acc");
		}				   
		k_sleep(K_SECONDS(sampling_interval));
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
		if (current_conn != NULL) {
			send_sensor_value(&value.temp, 1, "temp");
			send_sensor_value(&value.press, 1, "press");
			send_sensor_value(&value.humidity, 1, "humidity");
			send_sensor_value(&value.gas_res, 1, "gas_res");
			send_sensor_value(value.acc, 3, "acc");
			send_sensor_value(value.gyr, 3, "gyr");
		}					   
		k_sleep(K_SECONDS(sampling_interval));
	}

#endif
}

K_THREAD_DEFINE(sensor_data_collector_id, SENSOR_THREAD_STACKSIZE, sensor_data_collector, NULL,
		NULL, NULL, SENSOR_THREAD_PRIORITY, 0, 1000);