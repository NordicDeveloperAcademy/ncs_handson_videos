#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>

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