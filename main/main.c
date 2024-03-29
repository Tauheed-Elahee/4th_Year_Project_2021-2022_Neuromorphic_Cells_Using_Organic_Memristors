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

// MAC addr
#include "esp_mac.h"

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

    /* Temperature Sensor */
    configure_temp_sensor();
    read_temp_sensor();

    uint8_t base_mac_addr_array[6] = {0x00, 0x0d, 0x3f, 0xcd, 0x02, 0x5f};

    ESP_ERROR_CHECK(esp_efuse_mac_get_default(&base_mac_addr_array));
    for (int byte_index = 0; byte_index < 6; byte_index++) {
        ESP_LOGI(TAG, "MAC Address: %X", base_mac_addr_array[byte_index]);
    }

    mcp4542t_write_wiper_setting(MCP4542T_BASE_ADDR, 0, 0x080);
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
