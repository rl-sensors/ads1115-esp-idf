
#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_types.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2c_master.h"
#include "ads1115.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_ADS1115_MAX_TRANS_UNIT (48)
// Different ADS1115 device might share one I2C bus

static const char TAG[] = "i2c-ads1115";

esp_err_t i2c_ads1115_init(i2c_master_bus_handle_t bus_handle, const i2c_ads1115_config_t *ads1115_config, i2c_ads1115_handle_t *ads1115_handle)
{
    esp_err_t ret = ESP_OK;
    i2c_ads1115_handle_t out_handle;
    out_handle = (i2c_ads1115_handle_t)calloc(1, sizeof(*out_handle));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c ads1115 device");

    i2c_device_config_t i2c_dev_conf = {
        .scl_speed_hz = ads1115_config->ads1115_device.scl_speed_hz,
        .device_address = ads1115_config->ads1115_device.device_address,
    };

    if (out_handle->i2c_dev == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus_handle, &i2c_dev_conf, &out_handle->i2c_dev), err, TAG, "i2c new bus failed");
    }

    out_handle->buffer = (uint8_t*)calloc(1, ads1115_config->addr_wordlen + I2C_ADS1115_MAX_TRANS_UNIT);
    ESP_GOTO_ON_FALSE(out_handle->buffer, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c ads1115 device buffer");

    out_handle->addr_wordlen = ads1115_config->addr_wordlen;
    out_handle->write_time_ms = ads1115_config->write_time_ms;
    *ads1115_handle = out_handle;

    return ESP_OK;

err:
    if (out_handle && out_handle->i2c_dev) {
        i2c_master_bus_rm_device(out_handle->i2c_dev);
    }
    free(out_handle);
    return ret;
}

esp_err_t i2c_ads1115_write(i2c_ads1115_handle_t ads1115_handle, uint8_t reg_addr, const uint8_t *data, uint32_t size) {
    ESP_RETURN_ON_FALSE(ads1115_handle, ESP_ERR_NO_MEM, TAG, "no mem for buffer");
//    for (int i = 0; i < ads1115_handle->addr_wordlen; i++) {
//        ads1115_handle->buffer[i] = (address & (0xff << ((ads1115_handle->addr_wordlen - 1 - i) * 8))) >> ((ads1115_handle->addr_wordlen - 1 - i) * 8);
//    }
    ads1115_handle->buffer[0] = reg_addr;
    memcpy(ads1115_handle->buffer + 1, data, size);

    return i2c_master_transmit(ads1115_handle->i2c_dev, ads1115_handle->buffer, size + 1, -1);
}

esp_err_t i2c_ads1115_write_two_bytes(i2c_ads1115_handle_t ads1115_handle, uint8_t reg_addr, const uint16_t *data, uint32_t size) {
    ESP_RETURN_ON_FALSE(ads1115_handle, ESP_ERR_NO_MEM, TAG, "no mem for buffer");
//    for (int i = 0; i < ads1115_handle->addr_wordlen; i++) {
//        ads1115_handle->buffer[i] = (address & (0xff << ((ads1115_handle->addr_wordlen - 1 - i) * 8))) >> ((ads1115_handle->addr_wordlen - 1 - i) * 8);
//    }
    ads1115_handle->buffer[0] = reg_addr;
//    memcpy(ads1115_handle->buffer + 1, data, size);
    ads1115_handle->buffer[1] = *data >> 8;
    ads1115_handle->buffer[2] = *data & 0xFF;
    ESP_LOGI(TAG, "Sending data: %d, %d, %d", ads1115_handle->buffer[0], ads1115_handle->buffer[1], ads1115_handle->buffer[2]);

    return i2c_master_transmit(ads1115_handle->i2c_dev, ads1115_handle->buffer, size + 1, -1);
}

esp_err_t i2c_ads1115_read(i2c_ads1115_handle_t ads1115_handle, uint8_t address, uint8_t *data, uint32_t size) {
    return i2c_master_transmit_receive(ads1115_handle->i2c_dev, &address, 1, data, size, -1);
}

void i2c_ads1115_wait_idle(i2c_ads1115_handle_t ads1115_handle)
{
    // This is time for ADS1115 Self-Timed Write Cycle
    vTaskDelay(pdMS_TO_TICKS(ads1115_handle->write_time_ms));
}
