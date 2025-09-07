#include "esp_zigbee_core.h"
#include "esp_check.h"

static const int ZB_LOCK_TICKS = pdMS_TO_TICKS(1000);

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(
        esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK,
        , "ZB_HELPERS", "Failed to start Zigbee bdb commissioning");
}

static void zb_configure_reporting(void)
{
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
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };

    esp_err_t err = esp_zb_zcl_update_reporting_info(&temp_reporting_info);
    switch (err)
    {
    case ESP_OK:
        break;
    default:
        ESP_LOGW("ZB_HELPERS", "Updating temperature reporting info failed (status: %s)",
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
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };

    err = esp_zb_zcl_update_reporting_info(&hum_reporting_info);

    switch (err)
    {
    case ESP_OK:
        break;
    default:
        ESP_LOGW("ZB_HELPERS", "Updating humidity reporting info failed (status: %s)",
                 esp_err_to_name(err));
        break;
    }
}

static int16_t zb_temperature_to_s16(float temp)
{
    return (int16_t)(temp * 100);
}

static int16_t zb_to_s16(int temp)
{
    return (int16_t)(temp * 100);
}