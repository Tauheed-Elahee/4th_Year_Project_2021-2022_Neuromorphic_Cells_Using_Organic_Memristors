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

// Custom components
// ESP32 C3 I2C
#include "esp32c3_i2c.h"
// MCP4542T
#include "mcp4542t.h"

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

void app_main(void)
{

    /* Configure the peripheral according to the LED type */
    configure_led();
    configure_temp_sensor();
    read_temp_sensor();

    uint16_t wiper_setting = mcp4542t_read_wiper_setting(MCP4542T_BASE_ADDR, 0);

    ESP_LOGI(TAG, "NON-VOLATILE-0 in Hex: %X", wiper_setting);
    ESP_LOGI(TAG, "NON-VOLATILE-0 in Dec: %d", wiper_setting);

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
