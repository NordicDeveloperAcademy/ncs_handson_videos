#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include "sensor_data_collector.h"
#define STORAGE_THREAD_STACKSIZE 1024
#define STORAGE_THREAD_PRIORITY	 7
#define MTX_LOCK_WAIT_TIME	 K_MSEC(200)

/* Partition selection for where to store the data */
#define NVS_PARTITION		 sensor_data_in_partition
#define NVS_PARTITION_DEVICE	 FIXED_PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET	 FIXED_PARTITION_OFFSET(NVS_PARTITION)
#define NVS_PARTITION_SIZE	 FIXED_PARTITION_SIZE(NVS_PARTITION)
#define DATA_SIZE_MSG		 "use uint16_t little endian as input"
#define NO_DATA_MSG		 "No data"
#define ENV_NVS_ID		 0x859
#define BT_UUID_PSN_VAL BT_UUID_128_ENCODE(0xde550000, 0xc9f9, 0x4f0d, 0xa6e1, 0x766437949322)

#define BT_UUID_PSN_READ_REQ_VAL                                                                   \
	BT_UUID_128_ENCODE(0xde550001, 0xacb6, 0x4c73, 0x8445, 0x2563acbb43c2)

#define BT_UUID_PSN_READ_VAL	 BT_UUID_128_ENCODE(0xde550002, 0xacb6, 0x4c73, 0x8445, 0x2563acbb43c2)
#define BT_UUID_PSN_SERVICE	 BT_UUID_DECLARE_128(BT_UUID_PSN_VAL)
#define BT_UUID_PSN_READ_REQ	 BT_UUID_DECLARE_128(BT_UUID_PSN_READ_REQ_VAL)
#define BT_UUID_PSN_READ	 BT_UUID_DECLARE_128(BT_UUID_PSN_READ_VAL)
#define TMP_BUF_SIZE		 64
#define XFER_BUF_SIZE		 250

static K_MUTEX_DEFINE(nvs_mtx);
static uint32_t latest_id;
static struct nvs_fs fs = {
	.flash_device = NVS_PARTITION_DEVICE,
	.offset = NVS_PARTITION_OFFSET,
};

/* environmental sensor data storage chunk*/
struct env_storage_chunk {
	uint32_t id;
	struct sensor_value values[ENV_VALUES_CNT];
};
static char xfer_buf[XFER_BUF_SIZE];
static int xfer_cnt;
static int xfer_read_id;
static uint16_t xfer_depth;



static ssize_t bt_psn_data_req(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			       const void *buf, uint16_t len, uint16_t offset, uint8_t flags);

BT_GATT_SERVICE_DEFINE(psn_env_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_PSN_SERVICE),
		       BT_GATT_CHARACTERISTIC(BT_UUID_PSN_READ, BT_GATT_CHRC_NOTIFY,
					      BT_GATT_PERM_READ, NULL, NULL, NULL),
		       BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
		       BT_GATT_CUD("Stored Env. Data", BT_GATT_PERM_READ),
		       BT_GATT_CHARACTERISTIC(BT_UUID_PSN_READ_REQ, BT_GATT_CHRC_WRITE,
					      BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, NULL,
					      bt_psn_data_req, NULL),
		       BT_GATT_CCC(NULL, BT_GATT_PERM_READ),
		       BT_GATT_CUD("Stored Env. Data Request", BT_GATT_PERM_READ), );

static const struct bt_gatt_attr *psn_read_attr = &psn_env_svc.attrs[2];

size_t env_get_history(char *print_buf, const uint16_t depth)
{
	struct env_storage_chunk env_chunk;

	k_mutex_lock(&nvs_mtx, MTX_LOCK_WAIT_TIME);
	int rc = nvs_read_hist(&fs, ENV_NVS_ID, &env_chunk, sizeof(env_chunk), depth);

	k_mutex_unlock(&nvs_mtx);

	if (rc < 0) {
		return -EOVERFLOW;
	}
	int len = sprintf(print_buf, "%d ", env_chunk.id);

	for (size_t i = 0; i < ENV_VALUES_CNT; i++) {
		float tmp = sensor_value_to_float(&env_chunk.values[i]);

		len += sprintf(print_buf + len, "%.6f ", (double)tmp);
	}

	return len;
}

static void send_tail(struct bt_conn *conn, void *user_data)
{
	ARG_UNUSED(user_data);

	char tmp_buf[TMP_BUF_SIZE];
	int pos = 0;

	if (latest_id == 0) {
		bt_gatt_notify(conn, psn_read_attr, NO_DATA_MSG, sizeof(NO_DATA_MSG) - 1);
		printk("%s\n", NO_DATA_MSG);
	}

	while (xfer_cnt < xfer_depth) {
		int len = env_get_history(tmp_buf, (latest_id - xfer_read_id));

		if (len < 0) {
			xfer_depth = xfer_cnt;
			break;
		}

		if ((XFER_BUF_SIZE - pos) > len) {
			memcpy(xfer_buf + pos, tmp_buf, len);
			pos += len;
			xfer_read_id--;
			xfer_cnt++;
		} else {
			break;
		}
	}

	if (pos > 0) {
		struct bt_gatt_notify_params params = {
			.attr = psn_read_attr,
			.data = xfer_buf,
			.len = pos,
			.func = send_tail,
		};
		bt_gatt_notify_cb(conn, &params);
	}
}

static ssize_t bt_psn_data_req(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			       const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	if (len != sizeof(uint16_t)) {
		bt_gatt_notify(conn, psn_read_attr, DATA_SIZE_MSG, sizeof(DATA_SIZE_MSG) - 1);
		printk("%s\n", DATA_SIZE_MSG);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	xfer_cnt = 0;
	xfer_depth = *((uint16_t *)buf);

	xfer_read_id = latest_id;
	send_tail(conn, NULL);

	return 0;
}
static void sensor_data_storage(void)
{
	struct k_msgq *sensor_queue;
	sensorsreadings value;
	struct env_storage_chunk latest_env;
	struct flash_pages_info info;

	if (!device_is_ready(fs.flash_device)) {
		printk("Flash device %s is not ready.\n", fs.flash_device->name);
		return;
	}
	int rc = flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);

	if (rc) {
		printk("Unable to get page info.\n");
		return;
	}
	fs.sector_size = info.size;
	fs.sector_count = NVS_PARTITION_SIZE / fs.sector_size;

	rc = nvs_mount(&fs);
	if (rc) {
		printk("Flash Init failed.\n");
		return;
	}

	rc = nvs_read(&fs, ENV_NVS_ID, &latest_env, sizeof(latest_env));
	if (rc > 0) {
		printk("NVS latest id %d\n", latest_env.id);
		latest_id = latest_env.id;
	} else {
		latest_id = 0;
	}

	sensor_queue = get_sensor_readings_queue();
	if (sensor_queue == NULL) {
		printk("Error getting samples queue\n");
	}
	while (1) {

		int ret = k_msgq_get(sensor_queue, &value, K_FOREVER);
		if (ret) {
			printk("Return value from k_msgq_get = %d\n", ret);
		}
		// Populate the latest event
		latest_env.id = ++latest_id;
		latest_env.values[0] = value.temp;
		latest_env.values[1] = value.press;
		latest_env.values[2] = value.humidity;
#ifndef CONFIG_SIMULATED_SENSOR
		latest_env.values[3] = value.gas_res;
#endif
		k_mutex_lock(&nvs_mtx, MTX_LOCK_WAIT_TIME);
		nvs_write(&fs, ENV_NVS_ID, &latest_env, sizeof(latest_env));
		k_mutex_unlock(&nvs_mtx);
	}
}

K_THREAD_DEFINE(sensor_data_storage_id, STORAGE_THREAD_STACKSIZE, sensor_data_storage, NULL, NULL,
		NULL, STORAGE_THREAD_PRIORITY, 0, 2000);