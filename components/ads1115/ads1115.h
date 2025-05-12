#include <stdint.h>
#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ADS1115_ADDRESS_ADDR_GND    0x48 // address pin low (GND)
#define ADS1115_ADDRESS_ADDR_VDD    0x49 // address pin high (VCC)
#define ADS1115_ADDRESS_ADDR_SDA    0x4A // address pin tied to SDA pin
#define ADS1115_ADDRESS_ADDR_SCL    0x4B // address pin tied to SCL pin
#define ADS1115_DEFAULT_ADDRESS     ADS1115_ADDRESS_ADDR_GND

typedef enum { // register address
    ADS1115_CONVERSION_REGISTER_ADDR = 0,
    ADS1115_CONFIG_REGISTER_ADDR,
    ADS1115_LO_THRESH_REGISTER_ADDR,
    ADS1115_HI_THRESH_REGISTER_ADDR,
    ADS1115_MAX_REGISTER_ADDR
} ads1115_register_addresses_t;

typedef enum { // multiplex options
    ADS1115_MUX_0_1 = 0,
    ADS1115_MUX_0_3,
    ADS1115_MUX_1_3,
    ADS1115_MUX_2_3,
    ADS1115_MUX_0_GND,
    ADS1115_MUX_1_GND,
    ADS1115_MUX_2_GND,
    ADS1115_MUX_3_GND,
} ads1115_mux_t;

typedef enum { // full-scale resolution options
    ADS1115_FSR_6_144 = 0,
    ADS1115_FSR_4_096,
    ADS1115_FSR_2_048,
    ADS1115_FSR_1_024,
    ADS1115_FSR_0_512,
    ADS1115_FSR_0_256,
} ads1115_fsr_t;

typedef enum { // samples per second
    ADS1115_SPS_8 = 0,
    ADS1115_SPS_16,
    ADS1115_SPS_32,
    ADS1115_SPS_64,
    ADS1115_SPS_128,
    ADS1115_SPS_250,
    ADS1115_SPS_475,
    ADS1115_SPS_860
} ads1115_sps_t;

typedef enum {
    ADS1115_MODE_CONTINUOUS = 0,
    ADS1115_MODE_SINGLE
} ads1115_mode_t;

typedef union { // configuration register
    struct {
        uint16_t COMP_QUE:2;  // bits 0..  1  Comparator queue and disable
        uint16_t COMP_LAT:1;  // bit  2       Latching Comparator
        uint16_t COMP_POL:1;  // bit  3       Comparator Polarity
        uint16_t COMP_MODE:1; // bit  4       Comparator Mode
        uint16_t DR:3;        // bits 5..  7  Data rate
        uint16_t MODE:1;      // bit  8       Device operating mode
        uint16_t PGA:3;       // bits 9..  11 Programmable gain amplifier configuration
        uint16_t MUX:3;       // bits 12.. 14 Input multiplexer configuration
        uint16_t OS:1;        // bit  15      Operational status or single-shot conversion start
    } bit;
    uint16_t reg;
} ADS1115_CONFIG_REGISTER_Type;

typedef struct {
    i2c_device_config_t ads1115_device;  /*!< Configuration for ads1115 device */
    uint8_t write_time_ms;              /*!< ads1115 write time, typically 10ms*/
} i2c_ads1115_config_t;

struct i2c_ads1115_t {
    i2c_master_dev_handle_t i2c_dev;      /*!< I2C device handle */
    uint8_t *buffer;                      /*!< I2C transaction buffer */
    uint8_t write_time_ms;                /*!< I2C ads1115 write time(ms)*/
};

typedef struct i2c_ads1115_t i2c_ads1115_t;

/* handle of ADS1115 device */
typedef struct i2c_ads1115_t *i2c_ads1115_handle_t;

/**
 * @brief Init an ADS1115 device.
 *
 * @param[in] bus_handle I2C master bus handle
 * @param[in] ads1115_config Configuration of ADS1115
 * @param[out] ads1115_handle Handle of ADS1115
 * @return ESP_OK: Init success. ESP_FAIL: Not success.
 */
esp_err_t i2c_ads1115_init(i2c_master_bus_handle_t bus_handle, const i2c_ads1115_config_t *ads1115_config, i2c_ads1115_handle_t *ads1115_handle);

esp_err_t i2c_ads1115_write_two_bytes(i2c_ads1115_handle_t ads1115_handle, uint8_t address, const uint16_t *data, uint32_t size);

/**
 * @brief Read data from ADS1115
 *
 * @param ads1115_handle ADS1115 handle
 * @param address Block address inside ADS1115
 * @param data Data read from ADS1115
 * @param size Data read size
 * @return ESP_OK: Read success. Otherwise failed, please check I2C function fail reason.
 */
esp_err_t i2c_ads1115_read(i2c_ads1115_handle_t ads1115_handle, uint8_t address, uint8_t *data, uint32_t size);
/**
 * @brief Wait ads1115 finish. Typically 5ms
 *
 * @param ads1115_handle ADS1115 handle
 */
void i2c_ads1115_wait_idle(i2c_ads1115_handle_t ads1115_handle);

float fsr_multiplier(ADS1115_CONFIG_REGISTER_Type config);

#ifdef __cplusplus
}
#endif
