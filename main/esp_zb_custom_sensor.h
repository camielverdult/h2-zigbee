#include "esp_zigbee_core.h"
#include "esp_mac.h"

/**
 * @brief Zigbee HA standard temperature sensor clusters.
 *
 */
typedef struct zb_custom_cfg_s
{
    esp_zb_basic_cluster_cfg_t basic_cfg;                /*!<  Basic cluster configuration, @ref esp_zb_basic_cluster_cfg_s */
    esp_zb_identify_cluster_cfg_t identify_cfg;          /*!<  Identify cluster configuration, @ref esp_zb_identify_cluster_cfg_s */
    esp_zb_temperature_meas_cluster_cfg_t temp_meas_cfg; /*!<  Temperature measurement cluster configuration, @ref esp_zb_temperature_meas_cluster_cfg_s */
    esp_zb_humidity_meas_cluster_cfg_t humidity_meas_cfg; /*!<  Humidity measurement cluster configuration, @ref esp_zb_humidity_meas_cluster_cfg_s */
} zb_custom_cfg_t;

// /**
//  * @brief Zigbee HA standard temperature sensor device default config value.
//  *
//  */
#define ESP_ZB_DEFAULT_CUSTOM_SENSOR_CONFIG()                                             \
    {                                                                                     \
        .basic_cfg =                                                                      \
            {                                                                             \
                .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,                \
                .power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_DEFAULT_VALUE,              \
            },                                                                            \
        .identify_cfg =                                                                   \
            {                                                                             \
                .identify_time = ESP_ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE,         \
            },                                                                            \
        .temp_meas_cfg =                                                                  \
            {                                                                             \
                .measured_value = ESP_ZB_ZCL_TEMP_MEASUREMENT_MEASURED_VALUE_DEFAULT,     \
                .min_value = ESP_ZB_ZCL_TEMP_MEASUREMENT_MIN_MEASURED_VALUE_DEFAULT,      \
                .max_value = ESP_ZB_ZCL_TEMP_MEASUREMENT_MAX_MEASURED_VALUE_DEFAULT,      \
            },                                                                            \
        .humidity_meas_cfg =                                                              \
        {                                                                                 \
            .measured_value = ESP_ZB_ZCL_REL_HUMIDITY_MEASUREMENT_MEASURED_VALUE_DEFAULT, \
            .min_value = ESP_ZB_ZCL_REL_HUMIDITY_MEASUREMENT_MIN_MEASURED_VALUE_DEFAULT,  \
            .max_value = ESP_ZB_ZCL_REL_HUMIDITY_MEASUREMENT_MAX_MEASURED_VALUE_DEFAULT,  \
        }                                                                                 \
    }

static uint8_t efuse_mac[8];

// string to hold the efuse mac value and present it to ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID
#define MODEL_CHARS 24
static char UNIQUE_ID[MODEL_CHARS];

/// @brief Create cluster list for the custom temperature sensor endpoint
/// @param sensor_cfg Pointer to the custom sensor configuration structure, containing the configuration for the various clusters
/// @return Pointer to the created cluster list, containing the clusters required for the various Zigbee HA sensor functionality
static esp_zb_cluster_list_t *create_cluster_list(zb_custom_cfg_t *sensor_cfg)
{
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();

    // Basic
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(
        &sensor_cfg->basic_cfg);

    esp_err_t efuse_err = esp_efuse_mac_get_default(efuse_mac);

    memset(UNIQUE_ID, 0, MODEL_CHARS);
    if (efuse_err == ESP_OK) {
        // Get target type (esp32-h2, etc.)
        const int len = snprintf(UNIQUE_ID + 1, MODEL_CHARS - 1, CONFIG_IDF_TARGET " #%02X%02X%02X", efuse_mac[3], efuse_mac[4], efuse_mac[5]);
        assert(len < MODEL_CHARS); // Ensure we did not exceed buffer
        UNIQUE_ID[0] = (uint8_t)len;  // Set length prefix
    } else {
        ESP_LOGW("ZB_SENSOR", "Could not read MAC from efuse, err: %d", efuse_err);
        strcpy(UNIQUE_ID + 1, "UNKNOWN");
        UNIQUE_ID[0] = 7;  // Length of "UNKNOWN"
    }

    ESP_LOGI("ZB_SENSOR", "Setting ATTR_BASIC_MODEL_IDENTIFIER_ID to %s", UNIQUE_ID + 1);

    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, MANUFACTURER_NAME));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, UNIQUE_ID));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    // Identify
    ESP_ERROR_CHECK(
        esp_zb_cluster_list_add_identify_cluster(
            cluster_list,
            esp_zb_identify_cluster_create(&sensor_cfg->identify_cfg),
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    ESP_ERROR_CHECK(
        esp_zb_cluster_list_add_identify_cluster(
            cluster_list, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY),
            ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));

    // Temperature
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_temperature_meas_cluster(
        cluster_list,
        esp_zb_temperature_meas_cluster_create(&sensor_cfg->temp_meas_cfg),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    // Humidity
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_humidity_meas_cluster(
        cluster_list,
        esp_zb_humidity_meas_cluster_create(&sensor_cfg->humidity_meas_cfg),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    return cluster_list;
}

static esp_zb_ep_list_t *create_endpoint(uint8_t endpoint_id,
                                         zb_custom_cfg_t *sensor_cfg)
{
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();

    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = endpoint_id,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
        .app_device_version = 0};

    // Add temperature endpoint
    ESP_ERROR_CHECK(
        esp_zb_ep_list_add_ep(ep_list, create_cluster_list(sensor_cfg),
            endpoint_config)
    );

    return ep_list;
}