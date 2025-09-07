#include "esp_zigbee_core.h"

// /**
//  * @brief Zigbee HA standard temperature sensor device default config value.
//  *
//  */
#define ESP_ZB_DEFAULT_HUMIDITY_SENSOR_CONFIG()                                             \
    {                                                                                       \
        .basic_cfg =                                                                        \
            {                                                                               \
                .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,                  \
                .power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_DEFAULT_VALUE,                \
            },                                                                              \
        .identify_cfg =                                                                     \
            {                                                                               \
                .identify_time = ESP_ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE,           \
            },                                                                              \
        .humidity_meas_cfg =                                                                \
        {                                                                                   \
            .measured_value = ESP_ZB_ZCL_REL_HUMIDITY_MEASUREMENT_MEASURED_VALUE_DEFAULT,   \
            .min_value = ESP_ZB_ZCL_REL_HUMIDITY_MEASUREMENT_MIN_MEASURED_VALUE_DEFAULT,    \
            .max_value = ESP_ZB_ZCL_REL_HUMIDITY_MEASUREMENT_MAX_MEASURED_VALUE_DEFAULT,    \
        }                                                                                   \
    }

static esp_zb_cluster_list_t *create_cluster_list(esp_zb_temperature_sensor_cfg_t *temp_cfg, esp_zb_humidity_meas_cluster_cfg_t *humidity_meas_cfg)
{
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();

    // Basic
    esp_zb_attribute_list_t *basic_cluster = 
        esp_zb_basic_cluster_create(&temp_cfg->basic_cfg);

    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, MANUFACTURER_NAME));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, MODEL_IDENTIFIER));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    // Identify
    ESP_ERROR_CHECK(
        esp_zb_cluster_list_add_identify_cluster(
            cluster_list,
            esp_zb_identify_cluster_create(&temp_cfg->identify_cfg),
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    ESP_ERROR_CHECK(
        esp_zb_cluster_list_add_identify_cluster(
            cluster_list, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY),
            ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));

    // Temperature
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_temperature_meas_cluster(
        cluster_list,
        esp_zb_temperature_meas_cluster_create(&temp_cfg->temp_meas_cfg),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    // Humidity
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_humidity_meas_cluster(
        cluster_list,
        esp_zb_humidity_meas_cluster_create(humidity_meas_cfg),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    return cluster_list;
}

static esp_zb_ep_list_t *create_endpoint(uint8_t endpoint_id,
                                         esp_zb_temperature_sensor_cfg_t *temperature_sensor,
                                         esp_zb_humidity_meas_cluster_cfg_t *humidity_meas_cfg)
{
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();

    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = endpoint_id,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID,
        .app_device_version = 0};

    // Add temperature endpoint
    ESP_ERROR_CHECK(
        esp_zb_ep_list_add_ep(
            ep_list, create_cluster_list(temperature_sensor, humidity_meas_cfg),
            endpoint_config)
    );

    return ep_list;
}