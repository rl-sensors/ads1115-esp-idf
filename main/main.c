/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "ads1115.h"

#define SCL_IO_PIN CONFIG_I2C_MASTER_SCL
#define SDA_IO_PIN CONFIG_I2C_MASTER_SDA
#define MASTER_FREQUENCY CONFIG_I2C_MASTER_FREQUENCY
#define PORT_NUMBER 0

const char *TAG = "ADS1115";

static void disp_buf(uint8_t *buf, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        printf("%02x ", buf[i]);
        if ((i + 1) % 16 == 0) {
            printf("\n");
        }
    }
    printf("\n");
}

void app_main(void)
{
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = PORT_NUMBER,
        .scl_io_num = SCL_IO_PIN,
        .sda_io_num = SDA_IO_PIN,
        .glitch_ignore_cnt = 7,
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));

    i2c_ads1115_config_t ads1115_config = {
        .ads1115_device.scl_speed_hz = MASTER_FREQUENCY,
        .ads1115_device.device_address = ADS1115_DEFAULT_ADDRESS,
        .addr_wordlen = 1,
        .write_time_ms = 10,
    };

    i2c_ads1115_handle_t ads1115_handle;
    ESP_ERROR_CHECK(i2c_ads1115_init(bus_handle, &ads1115_config, &ads1115_handle));

    vTaskDelay(pdMS_TO_TICKS(1000));

    // Init the ADS1115
    // FSR2.048, 475SPS,
    const uint16_t config = 0b1000010011010011;

    uint8_t config_buf[2] = "";
    ESP_ERROR_CHECK(i2c_ads1115_read(ads1115_handle, ADS1115_RA_CONFIG, config_buf, 2));
    ESP_LOGI(TAG, "Read initial config config:  %d, %d", config_buf[0], config_buf[1]);

    ESP_ERROR_CHECK(i2c_ads1115_write_two_bytes(ads1115_handle, ADS1115_RA_CONFIG, &config, sizeof(config)));
    ESP_LOGI(TAG, "Written configuration: %d", config);

    ESP_ERROR_CHECK(i2c_ads1115_read(ads1115_handle, ADS1115_RA_CONFIG, config_buf, 2));
    ESP_LOGI(TAG, "Read config: %d, %d", config_buf[0], config_buf[1]);

    ESP_ERROR_CHECK(i2c_ads1115_read(ads1115_handle, ADS1115_LO_THRESH_REGISTER_ADDR, config_buf, 2));
    ESP_LOGI(TAG, "Read lo: %d, %d", config_buf[0], config_buf[1]);

    ESP_ERROR_CHECK(i2c_ads1115_read(ads1115_handle, ADS1115_HI_THRESH_REGISTER_ADDR, config_buf, 2));
    ESP_LOGI(TAG, "Read hi: %d, %d", config_buf[0], config_buf[1]);

    uint16_t lo = 0x0000;
    uint16_t hi = 0xFFFF;
    ESP_ERROR_CHECK(i2c_ads1115_write_two_bytes(ads1115_handle, ADS1115_LO_THRESH_REGISTER_ADDR, &lo, 2));
    ESP_ERROR_CHECK(i2c_ads1115_write_two_bytes(ads1115_handle, ADS1115_HI_THRESH_REGISTER_ADDR, &hi, 2));
    ESP_LOGI(TAG, "Written lo configuration: %d", lo);
    ESP_LOGI(TAG, "Written hi configuration: %d", hi);


    /*
    setMultiplexer(ADS1115_MUX_P0_N1);
    setGain(ADS1115_PGA_2P048);
    setMode(ADS1115_MODE_SINGLESHOT);
    setRate(ADS1115_RATE_128);
    setComparatorMode(ADS1115_COMP_MODE_HYSTERESIS);
    setComparatorPolarity(ADS1115_COMP_POL_ACTIVE_LOW);
    setComparatorLatchEnabled(ADS1115_COMP_LAT_NON_LATCHING);
    setComparatorQueueMode(ADS1115_COMP_QUE_DISABLE);
     }!ChfQ-VG1]<;-c
     */

    uint8_t read_buf[2];
    int16_t raw = 0;
    float voltage = 0;
    const int16_t bits = (1L<<15)-1;

    vTaskDelay(pdMS_TO_TICKS(1000));

    while (1) {
//        ESP_ERROR_CHECK(i2c_ads1115_write_two_bytes(ads1115_handle, ADS1115_RA_CONFIG, &config, sizeof(config)));
//        vTaskDelay(pdMS_TO_TICKS(25));
        ESP_ERROR_CHECK(i2c_ads1115_read(ads1115_handle, ADS1115_RA_CONVERSION, read_buf, 2));
        raw = ((uint16_t)read_buf[0] << 8) | (uint16_t)read_buf[1];

        // for FSR of 101b (bit 11:9)
        // voltage = raw * 0.256f / (float)bits;
        // for FSR of 100b (bit 11:9)
        voltage = raw * 0.512f / (float)bits;
        // for FSR of 011b (bit 11:9)
        voltage = raw * 1.024f / (float)bits;
        // for FSR of 010b (bit 11:9)
        voltage = raw * 2.048f / (float)bits;
        ESP_LOGI(TAG, "Raw value %d => %d, %d, voltage: %f", raw, read_buf[0], read_buf[1], voltage);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
