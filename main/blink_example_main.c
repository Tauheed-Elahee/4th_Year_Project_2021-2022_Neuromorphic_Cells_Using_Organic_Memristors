/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"

// experiment with temperature sensor
#include "driver/temp_sensor.h"

// Add I2C support
#include "driver/i2c.h"

// Custom components
// ESP32 C3 I2C
#include "esp32c3_i2c.h"

// MCP454T Memory Map
//#include "mcp4542t.h"

static const char *TAG = "example";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_BLINK_GPIO

static uint8_t s_led_state = 0;

static void blink_led(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, s_led_state);
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

// temperature sensor settings
static temp_sensor_config_t temp_sensor = {
    .dac_offset = TSENS_DAC_L2,
    .clk_div = 6,
};

static float tsens_out = 0;

static void configure_temp_sensor(void)
{
    // Set temperature sensor configuration
    temp_sensor_set_config(temp_sensor);
    temp_sensor_start();
}

static void read_temp_sensor(void)
{
    temp_sensor_read_celsius(&tsens_out);
    ESP_LOGI(TAG, "The temperature of the C3 chip is: %2.2f", tsens_out);
}



// i2c with potentiometer
#define MCP4542T_ADDR 0b0101100






// Memory map
#define MCP4542T_VOLATILE_0_REG_ADDR 0x00
#define MCP4542T_VOLATILE_1_REG_ADDR 0x01

#define MCP4542T_NON_VOLATILE_0_REG_ADDR 0x02
#define MCP4542T_NON_VOLATILE_1_REG_ADDR 0x03

#define MCP4542T_VOLATILE_TCON_REG_ADDR 0x04
#define MCP4542T_STATUS_REG_ADDR 0x05

#define MCP4542T_DATA_EEPROM_START_ADDR 0x06
#define MCP4542T_DATA_EEPROM_END_ADDR 0x0E
#define MCP4542T_HV_DATA_EEPROM_START_ADDR 0x0F

// Commands
#define MCP4542T_COMMAND_WRITE_DATA 0b00
#define MCP4542T_COMMAND_INCREMENT_DATA 0b01
#define MCP4542T_COMMAND_DECREMENT_DATA 0b10
#define MCP4542T_COMMAND_READ_DATA 0b11





// https://pcbartists.com/firmware/esp32-firmware/esp32-i2c-repeated-start-esp-idf-example/
esp_err_t mcp4542t_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    TickType_t wait_period = (TickType_t)(I2C_MASTER_FREQ_MS / portTICK_PERIOD_MS);
    esp_err_t err = ESP_OK;
    i2c_cmd_handle_t read_cmd = i2c_cmd_link_create();

    err = i2c_master_start(read_cmd);
    if (err != ESP_OK) {
        goto end;
    }
    err = i2c_master_write_byte(read_cmd, MCP4542T_ADDR << 1 | I2C_MASTER_WRITE, true);
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
    err = i2c_master_write_byte(read_cmd, MCP4542T_ADDR << 1 | I2C_MASTER_READ, true);
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










































void app_main(void)
{

    /* Configure the peripheral according to the LED type */
    configure_led();
    configure_temp_sensor();
    read_temp_sensor();

    // Initialize I2C
    uint8_t data[2];
    i2c_config_t i2c_config = esp32c3_i2c_config_default();

    ESP_ERROR_CHECK(esp32c3_i2c_master_init(i2c_config));
    ESP_LOGI(TAG, "I2C initialized successfully.");

    // Read data from potentiometer
    ESP_ERROR_CHECK(mcp4542t_register_read(MCP4542T_NON_VOLATILE_0_REG_ADDR, data, 2));
    ESP_LOGI(TAG, "NON-VOLATILE-0: %X %X", data[1], data[0]);

    // Shutdown I2C
    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_PORT));
    ESP_LOGI(TAG, "I2C de-initialized successfully.");

    /*
    while (1) {
        ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");
        read_temp_sensor();
        blink_led();
        // Toggle the LED state
        s_led_state = !s_led_state;
        vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
    }
    */
}
