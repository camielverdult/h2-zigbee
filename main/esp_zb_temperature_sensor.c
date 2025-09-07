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

#define I2C_PORT_AUTO -1
#define BMX280_SDA_NUM GPIO_NUM_5
#define BMX280_SCL_NUM GPIO_NUM_4

#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig to compile sensor (End Device) source code.
#endif

static const char *TAG = "ESP_ZB_TEMP_SENSOR";

static const int ZB_LOCK_TICKS = pdMS_TO_TICKS(1000);

i2c_master_bus_handle_t i2c_bus_init(uint8_t sda_io, uint8_t scl_io) {
  i2c_master_bus_config_t i2c_bus_config = {
      .i2c_port = I2C_PORT_AUTO,
      .sda_io_num = sda_io,
      .scl_io_num = scl_io,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };
  i2c_master_bus_handle_t bus_handle;
  ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));
  ESP_LOGI("test", "I2C master bus created");
  return bus_handle;
}

static switch_func_pair_t button_func_pair[] = {
    {GPIO_INPUT_IO_TOGGLE_SWITCH, SWITCH_ONOFF_TOGGLE_CONTROL}};

static void esp_app_buttons_handler(switch_func_pair_t *button_func_pair) {
  if (button_func_pair->func == SWITCH_ONOFF_TOGGLE_CONTROL) {
    /* Send report attributes command */
    esp_zb_zcl_report_attr_cmd_t report_attr_cmd = {0};
    report_attr_cmd.address_mode =
        ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
    report_attr_cmd.attributeID = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID;
    report_attr_cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
    report_attr_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT;
    report_attr_cmd.zcl_basic_cmd.src_endpoint = HA_ESP_SENSOR_ENDPOINT;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd);
    esp_zb_lock_release();
    ESP_EARLY_LOGI(TAG, "Send 'report attributes' command");
  }
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask) {
  ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) ==
                          ESP_OK,
                      , TAG, "Failed to start Zigbee bdb commissioning");
}

static i2c_master_bus_handle_t bus_handle;
static bmx280_t *bmx280 = NULL;

esp_err_t bmx280_dev_init(bmx280_t **bmx280,
                          i2c_master_bus_handle_t bus_handle) {
  *bmx280 = bmx280_create_master(bus_handle);
  if (!*bmx280) {
    ESP_LOGE(TAG, "Could not create bmx280 driver.");
    return ESP_FAIL;
  }

  ESP_ERROR_CHECK(bmx280_init(*bmx280));
  bmx280_config_t bmx_cfg = BMX280_DEFAULT_CONFIG;
  ESP_ERROR_CHECK(bmx280_configure(*bmx280, &bmx_cfg));
  return ESP_OK;
}

typedef struct bme280_measurement_t {
  float temperature;
  int pressure;
  int humidity;
} bme280_measurement;

/** Temperature sensor callback
 *
 * @param[in] temperature temperature value in degrees Celsius from sensor
 *
 */
typedef void (*bme_sensor_callback_t)(bme280_measurement reading);

static bme_sensor_callback_t zigbee_update_callback_func = NULL;

/**
 * @brief Tasks for updating the sensor value
 *
 * @param arg      Unused value.
 */
static void temp_sensor_driver_value_update(void *arg) {
  bme280_measurement sens_value = {
      .temperature = 0.0f, .humidity = 0.0f, .pressure = 0.0f};
  for (;;) {
    if (zigbee_update_callback_func == NULL) {
      ESP_LOGE(TAG, "No callback function set yet, waiting...");
      vTaskDelay(pdMS_TO_TICKS(1 * 1000));

      continue;
    }

    //  For a next measurement, forced mode needs to be selected again.
    ESP_ERROR_CHECK(bmx280_setMode(bmx280, BMX280_MODE_FORCE));
    do {
      vTaskDelay(pdMS_TO_TICKS(100));
      if (bmx280_isSampling(bmx280)) {
        ESP_LOGI(TAG, "Waiting on bmx280 to finish sampling");
      }
    } while (bmx280_isSampling(bmx280));

    float temperature, pressure, humidity;

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

    zigbee_update_callback_func(sens_value);

    vTaskDelay(pdMS_TO_TICKS(5 * 1000));
  }
}

static void esp_app_temp_sensor_handler(bme280_measurement measurement) {
  ESP_LOGI(TAG, "Setting zigbee attributes");
  int16_t temp = zb_temperature_to_s16(measurement.temperature);

  if (!esp_zb_lock_acquire(ZB_LOCK_TICKS)) {
    ESP_LOGW(TAG, "Could not acquire zigbee lock");
    return;
  }

  esp_zb_zcl_set_attribute_val(
      HA_ESP_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
      ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
      &temp, false);

  esp_zb_lock_release();
}

static esp_err_t deferred_driver_init(void) {
  static bool is_inited = false;

  if (!is_inited) {
    bus_handle = i2c_bus_init(BMX280_SDA_NUM, BMX280_SCL_NUM);

    ESP_RETURN_ON_ERROR(bmx280_dev_init(&bmx280, bus_handle), TAG,
                        "Failed to initialize bme280");

    ESP_RETURN_ON_ERROR(bmx280_setMode(bmx280, BMX280_MODE_FORCE), TAG,
                        "Failed to set bme280 mode to forced");

    zigbee_update_callback_func = esp_app_temp_sensor_handler;

    esp_err_t sensor_task_res =
        (xTaskCreate(temp_sensor_driver_value_update, "bme280_update", 8192,
                     NULL, 10, NULL) == pdTRUE)
            ? ESP_OK
            : ESP_FAIL;
    ESP_RETURN_ON_ERROR(sensor_task_res, TAG,
                        "Failed to initialize temperature sensor task");

    ESP_RETURN_ON_FALSE(switch_driver_init(button_func_pair,
                                           PAIR_SIZE(button_func_pair),
                                           esp_app_buttons_handler),
                        ESP_FAIL, TAG, "Failed to initialize switch driver");
    is_inited = true;
  }
  return is_inited ? ESP_OK : ESP_FAIL;
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
      ESP_LOGI(TAG, "Deferred driver initialization %s",
               deferred_driver_init() ? "failed" : "successful");
      ESP_LOGI(TAG, "Device started up in%s factory-reset mode",
               esp_zb_bdb_is_factory_new() ? "" : " non");
      if (esp_zb_bdb_is_factory_new()) {
        ESP_LOGI(TAG, "Start network steering");
        esp_zb_bdb_start_top_level_commissioning(
            ESP_ZB_BDB_MODE_NETWORK_STEERING);
      } else {
        ESP_LOGI(TAG, "Device rebooted");
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

// static esp_zb_cluster_list_t *custom_temperature_sensor_clusters_create(
//     esp_zb_temperature_sensor_cfg_t *temperature_sensor) {
//   esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
//   esp_zb_attribute_list_t *basic_cluster =
//       esp_zb_basic_cluster_create(&(temperature_sensor->basic_cfg));
//   ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(
//       basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID,
//       MANUFACTURER_NAME));
//   ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(
//       basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,
//       MODEL_IDENTIFIER));
//   ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(
//       cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
//   ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(
//       cluster_list,
//       esp_zb_identify_cluster_create(&(temperature_sensor->identify_cfg)),
//       ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
//   ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(
//       cluster_list, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY),
//       ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));
//   ESP_ERROR_CHECK(esp_zb_cluster_list_add_temperature_meas_cluster(
//       cluster_list,
//       esp_zb_temperature_meas_cluster_create(
//           &(temperature_sensor->temp_meas_cfg)),
//       ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
//   return cluster_list;
// }

// static esp_zb_ep_list_t *custom_temperature_sensor_ep_create(
//     uint8_t endpoint_id, esp_zb_temperature_sensor_cfg_t *temperature_sensor)
// {
//     esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();

//     esp_zb_endpoint_config_t endpoint_config = {
//         .endpoint = endpoint_id,
//         .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
//         .app_device_id = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID,
//         .app_device_version = 0
//     };

//     esp_zb_ep_list_add_ep(
//         ep_list, custom_temperature_sensor_clusters_create(temperature_sensor),
//         endpoint_config);

//     return ep_list;
// }

static void esp_zb_task(void *pvParameters) {
    /* Initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    /* Create customized temperature sensor endpoint */
    esp_zb_temperature_sensor_cfg_t sensor_cfg =
        ESP_ZB_DEFAULT_TEMPERATURE_SENSOR_CONFIG();
    /* Set (Min|Max)MeasuredValure */
    sensor_cfg.temp_meas_cfg.min_value =
        zb_temperature_to_s16(ESP_TEMP_SENSOR_MIN_VALUE);
    sensor_cfg.temp_meas_cfg.max_value =
        zb_temperature_to_s16(ESP_TEMP_SENSOR_MAX_VALUE);

    esp_zb_humidity_meas_cluster_cfg_t humidity_meas_cfg = {
        .measured_value = ESP_ZB_ZCL_REL_HUMIDITY_MEASUREMENT_MEASURED_VALUE_DEFAULT,
        .min_value = zb_temperature_to_s16(ESP_HUM_SENSOR_MIN_VALUE),
        .max_value = zb_temperature_to_s16(ESP_HUM_SENSOR_MAX_VALUE),
    };

    esp_zb_ep_list_t *esp_zb_sensor_ep = create_endpoint(
        HA_ESP_SENSOR_ENDPOINT, &sensor_cfg, &humidity_meas_cfg);

    /* Register the device */
    esp_zb_device_register(esp_zb_sensor_ep);

    /* Config the reporting info  */
    esp_zb_zcl_reporting_info_t temp_reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_ESP_SENSOR_ENDPOINT,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 1,
        .u.send_info.max_interval = 0,
        .u.send_info.def_min_interval = 1,
        .u.send_info.def_max_interval = 0,
        .u.send_info.delta.u16 = 100,
        .attr_id = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
        //   .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };

    esp_err_t err = esp_zb_zcl_update_reporting_info(&temp_reporting_info);
    switch (err) {
    case ESP_OK:
        break;
    default:
        ESP_LOGW(TAG, "Updating temperature reporting info failed (status: %s)",
                esp_err_to_name(err));
        break;
    }

    esp_zb_zcl_reporting_info_t hum_reporting_info = {
      .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
      .ep = HA_ESP_SENSOR_ENDPOINT,
      .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
      .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
      .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
      .u.send_info.min_interval = 1,
      .u.send_info.max_interval = 0,
      .u.send_info.def_min_interval = 1,
      .u.send_info.def_max_interval = 0,
      .u.send_info.delta.u16 = 100,
      .attr_id = ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
      //   .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };

    switch (err)
    {
    case ESP_OK:
        break;
    default:
        ESP_LOGW(TAG, "Updating temperature reporting info failed (status: %s)",
                 esp_err_to_name(err));
        break;
    }

  err = esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
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

  /* Start Zigbee stack task */
  xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}
