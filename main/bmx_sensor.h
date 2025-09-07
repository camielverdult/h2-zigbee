#include "bmx280.h"
#include "driver/i2c_types.h"

#define I2C_PORT_AUTO -1
#define BMX280_SDA_NUM GPIO_NUM_5
#define BMX280_SCL_NUM GPIO_NUM_4

static bmx280_t *bmx280 = NULL;

esp_err_t bmx280_dev_init(bmx280_t **bmx280, i2c_master_bus_handle_t bus_handle)
{
    if (!bmx280 || !bus_handle)
    {
        ESP_LOGE("BMX280", "Invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }

    *bmx280 = bmx280_create_master(bus_handle);
    if (!*bmx280)
    {
        ESP_LOGE("BMX280", "Could not create bmx280 driver.");
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = bmx280_init(*bmx280);
    if (err != ESP_OK)
    {
        ESP_LOGE("BMX280", "Failed to initialize bmx280: %s", esp_err_to_name(err));
        *bmx280 = NULL;
        return err;
    }

    bmx280_config_t bmx_cfg = BMX280_DEFAULT_CONFIG;
    err = bmx280_configure(*bmx280, &bmx_cfg);
    if (err != ESP_OK)
    {
        ESP_LOGE("BMX280", "Failed to configure bmx280: %s", esp_err_to_name(err));
        *bmx280 = NULL;
        return err;
    }

    ESP_LOGI("BMX280", "BMX280 initialized successfully");
    return ESP_OK;
}

typedef struct bme280_measurement_t
{
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