// Public Dependencies
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/i2c.h"

// Default I2C config
#define I2C_MASTER_PORT 0
#define I2C_SDA_GPIO 6
#define I2C_SCL_GPIO 7
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_FREQ_HZ 400000
#define I2C_MASTER_FREQ_MS 1000
#define I2C_MASTER_CLK_FLAGS 0

i2c_config_t esp32c3_i2c_config
(
    i2c_mode_t i2c_mode,
    int i2c_SDA_gpio,
    bool i2c_SDA_pullup_en,
    int i2c_SCL_gpio,
    bool i2c_SCL_pullup_en,
    uint32_t i2c_master_clk_freq_hz,
    uint32_t i2c_clk_flags
);

i2c_config_t esp32c3_i2c_config_default(void);

esp_err_t esp32c3_i2c_master_init(i2c_config_t i2c_config);