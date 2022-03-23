// Public Dependencies
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/i2c.h"

// Custom Dependencies
#include "esp32c3_i2c.h"

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




// i2c with potentiometer
#define MCP4542T_BASE_ADDR 0b0101100





esp_err_t mcp4542t_register_read
(
    uint8_t dev_addr,
    uint8_t reg_addr,
    uint8_t *data,
    size_t len
);

uint16_t mcp4542t_read_wiper_setting (uint8_t dev_addr, bool wiper_0_or_1);