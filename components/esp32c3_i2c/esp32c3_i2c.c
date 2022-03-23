#include <stdio.h>
#include "esp32c3_i2c.h"

i2c_config_t esp32c3_i2c_config
(
    i2c_mode_t i2c_mode,
    int i2c_SDA_gpio,
    bool i2c_SDA_pullup_en,
    int i2c_SCL_gpio,
    bool i2c_SCL_pullup_en,
    uint32_t i2c_master_clk_freq_hz,
    uint32_t i2c_clk_flags
)
{
    i2c_config_t i2c_config = {
        .mode = i2c_mode,
        .sda_io_num = i2c_SDA_gpio,
        .sda_pullup_en = i2c_SDA_pullup_en,
        .scl_io_num = i2c_SCL_gpio,
        .scl_pullup_en = i2c_SCL_pullup_en,
        .master.clk_speed = i2c_master_clk_freq_hz,
        .clk_flags = i2c_clk_flags,
    };

    return i2c_config;
}

i2c_config_t esp32c3_i2c_config_default(void)
{
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_SCL_GPIO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = I2C_MASTER_CLK_FLAGS,
    };

    return i2c_config;
}

esp_err_t esp32c3_i2c_master_init(i2c_config_t i2c_config)
{
    i2c_param_config(I2C_MASTER_PORT, &i2c_config);

    return i2c_driver_install(I2C_MASTER_PORT, i2c_config.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}