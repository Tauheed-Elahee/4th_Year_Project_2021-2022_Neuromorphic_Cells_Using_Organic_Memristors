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


// Custom components
// Temperature Sensor // experiment with temperature sensor
#include "temperature.h"
// ESP32 C3 I2C
#include "esp32c3_i2c.h"
// MCP4542T
#include "mcp4542t.h"

static const char *TAG = "example";

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
