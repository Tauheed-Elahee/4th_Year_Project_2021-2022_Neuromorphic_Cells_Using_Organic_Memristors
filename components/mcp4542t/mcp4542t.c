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
    err = i2c_master_write_byte(read_cmd, reg_addr << 4 | MCP4542T_COMMAND_READ_DATA << 2 | 0b11, true);
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

uint16_t mcp4542t_read_wiper_setting(uint8_t dev_addr, bool wiper_0_or_1) {
    char *TAG = "example";

    uint8_t reg_addr = (wiper_0_or_1)? MCP4542T_NON_VOLATILE_1_REG_ADDR: MCP4542T_NON_VOLATILE_0_REG_ADDR;

    // Initialize I2C
    uint8_t data[2];
    i2c_config_t i2c_config = esp32c3_i2c_config_default();

    ESP_ERROR_CHECK(esp32c3_i2c_master_init(i2c_config));
    ESP_LOGI(TAG, "I2C initialized successfully.");

    // Read data from potentiometer
    ESP_ERROR_CHECK(mcp4542t_register_read(dev_addr, reg_addr, data, 2));
    ESP_LOGI(TAG, "NON-VOLATILE-0: %X %X", data[1], data[0]);

    // Shutdown I2C
    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_PORT));
    ESP_LOGI(TAG, "I2C de-initialized successfully.");

    uint16_t wiper_setting = data[1] << 4 | data[0];

    return wiper_setting;
}