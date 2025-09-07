/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * Zigbee HA_temperature_sensor Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */
#include "esp_zb_temperature_sensor.h"
#include "esp_zb_custom_sensor.h"
#include "zigbee_helpers.h"
#include "bmx_sensor.h"

#include "esp_err.h"
#include "freertos/projdefs.h"
#include "switch_driver.h"
#include "temp_sensor_driver.h"

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "nvs_flash.h"

#include "bmx280.h"
#include "driver/i2c_types.h"
#include <string.h>

#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig to compile sensor (End Device) source code.
#endif

static const char *TAG = "ESP_ZB_TEMP_SENSOR";

static i2c_master_bus_handle_t bus_handle;
static bool is_inited = false;

i2c_master_bus_handle_t i2c_bus_init(uint8_t sda_io, uint8_t scl_io) {
  i2c_master_bus_config_t i2c_bus_config = {
      .i2c_port = I2C_PORT_AUTO,
      .sda_io_num = sda_io,
      .scl_io_num = scl_io,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };

  ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));
  ESP_LOGI(TAG, "I2C master bus created");
  return bus_handle;
}

static void esp_app_temp_sensor_handler(bme280_measurement measurement) {
    ESP_LOGI(TAG, "Setting zigbee attributes");
    int16_t temp = zb_temperature_to_s16(measurement.temperature);
    int16_t hum = zb_to_s16(measurement.humidity);

    if (!esp_zb_lock_acquire(ZB_LOCK_TICKS))
    {
        ESP_LOGW(TAG, "Could not acquire zigbee lock");
        return;
    }

    esp_zb_zcl_status_t err = esp_zb_zcl_set_attribute_val(
        HA_ESP_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
        &temp, false);

    if (err != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGW(TAG, "Could not set temperature attribute, err: %d", err);
    }

    esp_zb_zcl_set_attribute_val(
        HA_ESP_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
        &hum, false);

    if (err != ESP_ZB_ZCL_STATUS_SUCCESS)
    {
        ESP_LOGW(TAG, "Could not set temperature attribute, err: %d", err);
    }

    esp_zb_lock_release();
}

/**
 * @brief Task for updating the sensor value
 *
 * @param arg      Unused value.
 */
static void sensor_update_task(void *arg)
{
    float temperature, pressure, humidity;
    
    bme280_measurement sens_value = {
        .temperature = 0.0f, .humidity = 0.0f, .pressure = 0.0f
    };

    while (1)
    {
        // For a (next) measurement, forced mode needs to be selected again.
        ESP_ERROR_CHECK(bmx280_setMode(bmx280, BMX280_MODE_FORCE));

        while (bmx280_isSampling(bmx280))
        {
            vTaskDelay(pdMS_TO_TICKS(500));
            ESP_LOGI(TAG, "Waiting on bmx280 to finish sampling");
        }

        ESP_ERROR_CHECK(
            bmx280_readoutFloat(bmx280, &temperature, &pressure, &humidity));

        // Convert pressure to hPa
        pressure /= 100.0;

        // Round values to 1 decimal place
        sens_value.temperature = roundf(temperature * 10.0) / 10.0;
        sens_value.pressure = round(pressure);
        sens_value.humidity = round(humidity);

        ESP_LOGI(TAG, "Read Values: temp = %f, pres = %i, hum = %i",
                 sens_value.temperature, sens_value.pressure, sens_value.humidity);

        esp_app_temp_sensor_handler(sens_value);

        vTaskDelay(pdMS_TO_TICKS(ESP_TEMP_SENSOR_UPDATE_INTERVAL * 1000));
    }
}

static esp_err_t deferred_driver_init(void) {
    if (is_inited) {
        return ESP_FAIL;
    }

    bool task_res = xTaskCreate(sensor_update_task, "bme280_update", 8192,
                                NULL, 10, NULL);

    ESP_RETURN_ON_FALSE(task_res == pdTRUE, ESP_FAIL, TAG,
                    "Failed to create temperature sensor update task");
    
    is_inited = true;

    return is_inited ? ESP_OK : ESP_FAIL;
}

void post_join(void) {
    // Configure after joining network
    esp_err_t success = deferred_driver_init();
    if (success == ESP_OK) {
        ESP_LOGI(TAG, "Deferred driver initialization successful");
    } else {
        ESP_LOGE(TAG, "Deferred driver initialization failed");
    }

    zb_configure_reporting();
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct) {
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Device started up in%s factory-reset mode",
                    esp_zb_bdb_is_factory_new() ? "" : " non");
        if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(
                    ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
                ESP_LOGI(TAG, "Device rebooted");
                post_join();
        }
        } else {
            ESP_LOGW(TAG, "%s failed with status: %s, retrying",
                    esp_zb_zdo_signal_to_string(sig_type),
                    esp_err_to_name(err_status));
            esp_zb_scheduler_alarm(
                (esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                ESP_ZB_BDB_MODE_INITIALIZATION, 1000);
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG,
                    "Joined network successfully (Extended PAN ID: "
                    "%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, "
                    "Channel:%d, Short Address: 0x%04hx)",
                    extended_pan_id[7], extended_pan_id[6], extended_pan_id[5],
                    extended_pan_id[4], extended_pan_id[3], extended_pan_id[2],
                    extended_pan_id[1], extended_pan_id[0], esp_zb_get_pan_id(),
                    esp_zb_get_current_channel(), esp_zb_get_short_address());

            // Configure after joining network
            post_join();

        } else {
            ESP_LOGI(
                TAG,
                "Network steering was not successful, not connected? (status: %s)",
                esp_err_to_name(err_status));
            esp_zb_scheduler_alarm(
                (esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s",
                esp_zb_zdo_signal_to_string(sig_type), sig_type,
                esp_err_to_name(err_status));
        break;
    }
}

static void esp_zb_task(void *pvParameters) {
    /* Initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    /* Create customized sensor endpoint */
    zb_custom_cfg_t sensor_cfg = 
        ESP_ZB_DEFAULT_CUSTOM_SENSOR_CONFIG();

    // Set the temperature sensor measurement range
    sensor_cfg.temp_meas_cfg.min_value =
        zb_to_s16(ESP_TEMP_SENSOR_MIN_VALUE);
    sensor_cfg.temp_meas_cfg.max_value =
        zb_to_s16(ESP_TEMP_SENSOR_MAX_VALUE);

    esp_zb_ep_list_t *esp_zb_sensor_ep = create_endpoint(
        HA_ESP_SENSOR_ENDPOINT, &sensor_cfg);

    /* Register the device */
    esp_zb_device_register(esp_zb_sensor_ep);

    esp_err_t err = esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    switch (err) {
    case ESP_OK:
        break;
    default:
        ESP_LOGW(TAG, "Setting primary network channel failed (status: %s)",
            esp_err_to_name(err));
        break;
    }

    ESP_ERROR_CHECK(esp_zb_start(false));

    esp_zb_stack_main_loop();
}

void app_main(void) {
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    bus_handle = i2c_bus_init(BMX280_SDA_NUM, BMX280_SCL_NUM);

    esp_err_t err = bmx280_dev_init(&bmx280, bus_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize bme280: %s", esp_err_to_name(err));
        return;
    }

    err = bmx280_setMode(bmx280, BMX280_MODE_FORCE);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set bme280 mode to forced: %s", esp_err_to_name(err));
        return;
    }

    /* Start Zigbee stack task */
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}
