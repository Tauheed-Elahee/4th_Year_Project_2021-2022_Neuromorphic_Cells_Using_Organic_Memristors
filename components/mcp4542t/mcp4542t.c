#include <stdio.h>
#include "mcp4542t.h"


// https://pcbartists.com/firmware/esp32-firmware/esp32-i2c-repeated-start-esp-idf-example/
esp_err_t mcp4542t_register_read
(
    uint8_t dev_addr,
    uint8_t reg_addr,
    uint8_t *data,
    size_t len
)
{
    TickType_t wait_period = (TickType_t)(I2C_MASTER_FREQ_MS / portTICK_PERIOD_MS);
    esp_err_t err = ESP_OK;
    i2c_cmd_handle_t read_cmd = i2c_cmd_link_create();

    err = i2c_master_start(read_cmd);
    if (err != ESP_OK) {
        goto end;
    }
    err = i2c_master_write_byte(read_cmd, dev_addr << 1 | I2C_MASTER_WRITE, true);
    if (err != ESP_OK) {
        goto end;
    }
    err = i2c_master_write_byte(read_cmd, reg_addr << 4 | MCP4542T_COMMAND_READ_DATA << 2 | 0b00, true);
    if (err != ESP_OK) {
        goto end;
    }
    err = i2c_master_start(read_cmd);
    if (err != ESP_OK) {
        goto end;
    }
    err = i2c_master_write_byte(read_cmd, dev_addr << 1 | I2C_MASTER_READ, true);
    if (err != ESP_OK) {
        goto end;
    }
    for (size_t offset = (len-1); offset > 0; offset--)
    {
        err = i2c_master_read(read_cmd, data + offset, 1, I2C_MASTER_ACK);
        if (err != ESP_OK) {
            goto end;
        }
    }
    err = i2c_master_read_byte(read_cmd, data, I2C_MASTER_LAST_NACK);
    if (err != ESP_OK) {
        goto end;
    }
    err = i2c_master_stop(read_cmd);
    if (err != ESP_OK) {
        goto end;
    }
    err = i2c_master_cmd_begin(I2C_MASTER_PORT, read_cmd, wait_period);
    if (err != ESP_OK) {
        goto end;
    }

end:
    i2c_cmd_link_delete(read_cmd);
    return err;
}

esp_err_t mcp4542t_register_write
(
    uint8_t dev_addr,
    uint8_t reg_addr,
    uint16_t data
)
{
    TickType_t wait_period = (TickType_t)(I2C_MASTER_FREQ_MS / portTICK_PERIOD_MS);
    esp_err_t err = ESP_OK;
    i2c_cmd_handle_t write_cmd = i2c_cmd_link_create();

    err = i2c_master_start(write_cmd);
    if (err != ESP_OK) {
        goto end;
    }
    err = i2c_master_write_byte(write_cmd, dev_addr << 1 | I2C_MASTER_WRITE, true);
    if (err != ESP_OK) {
        goto end;
    }
    err = i2c_master_write_byte(write_cmd, reg_addr << 4 | MCP4542T_COMMAND_WRITE_DATA << 2 | 0b0 << 1 | (data & 0x0100) >> 8, true);
    if (err != ESP_OK) {
        goto end;
    }
    err = i2c_master_write_byte(write_cmd, data & 0x0FF, true);
    if (err != ESP_OK) {
        goto end;
    }
    err = i2c_master_stop(write_cmd);
    if (err != ESP_OK) {
        goto end;
    }
    err = i2c_master_cmd_begin(I2C_MASTER_PORT, write_cmd, wait_period);
    if (err != ESP_OK) {
        goto end;
    }

end:
    i2c_cmd_link_delete(write_cmd);
    ets_delay_us(4000);
    return err;
}

uint16_t mcp4542t_read_wiper_setting(uint8_t dev_addr, bool wiper_0_or_1) {
    // Determine the correct register address of the requested wiper
    uint8_t reg_addr = (wiper_0_or_1)? MCP4542T_NON_VOLATILE_1_REG_ADDR: MCP4542T_NON_VOLATILE_0_REG_ADDR;

    // Initialize I2C
    uint8_t data[2];
    i2c_config_t i2c_config = esp32c3_i2c_config_default();

    // Initialize I2C bus
    esp32c3_i2c_master_init(i2c_config);
    // Read data from potentiometer
    mcp4542t_register_read(dev_addr, reg_addr, data, 2);
    // Shutdown I2C bus
    i2c_driver_delete(I2C_MASTER_PORT);

    // Collect buffer into a result
    uint16_t wiper_setting = data[1] << 4 | data[0];

    return wiper_setting;
}

void mcp4542t_write_wiper_setting (uint8_t dev_addr, bool wiper_0_or_1, uint16_t data) {
    // Determine the correct register address of the requested wiper
    uint8_t reg_addr = (wiper_0_or_1)? MCP4542T_NON_VOLATILE_1_REG_ADDR: MCP4542T_NON_VOLATILE_0_REG_ADDR;

    // Initialize I2C
    i2c_config_t i2c_config = esp32c3_i2c_config_default();

    // Initialize I2C bus
    esp32c3_i2c_master_init(i2c_config);
    // Read data from potentiometer
    mcp4542t_register_write(dev_addr, reg_addr, data);
    // Shutdown I2C bus
    i2c_driver_delete(I2C_MASTER_PORT);
}