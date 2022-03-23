#include <stdio.h>
#include "esp_log.h"
#include "driver/temp_sensor.h"
#include "temperature.h"

 
void configure_temp_sensor(void)
{
    // temperature sensor settings
    temp_sensor_config_t temp_sensor = {
        .dac_offset = TSENS_DAC_L2,
        .clk_div = 6,
    };

    // Set temperature sensor configuration
    temp_sensor_set_config(temp_sensor);
    temp_sensor_start();
}

void read_temp_sensor(void)
{
    char *TAG = "example";
    float tsens_out = 0;
    temp_sensor_read_celsius(&tsens_out);
    ESP_LOGI(TAG, "The temperature of the C3 chip is: %2.2f", tsens_out);
}