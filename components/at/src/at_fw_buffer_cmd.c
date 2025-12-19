/*
 * Custom AT Commands for Firmware Buffer OTA - Phase 1
 * 
 * This file implements memory diagnostic commands to verify PSRAM availability
 * before implementing the full OTA buffer system.
 * 
 * Commands implemented:
 *   AT+FWMEMINFO?  - Query available PSRAM and internal RAM
 * 
 * Installation:
 *   1. Copy this file to: esp-at/components/at/src/at_fw_buffer_cmd.c
 *   2. Add "src/at_fw_buffer_cmd.c" to components/at/CMakeLists.txt
 *   3. Add to force_symbol_ref.cmake:
 *      target_link_libraries(${COMPONENT_LIB} INTERFACE "-u esp_at_fw_buffer_cmd_regist")
 *   4. Rebuild and flash
 */

#include <stdio.h>
#include <string.h>
#include "esp_at.h"
#include "esp_at_cmd_register.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

static const char *TAG = "AT_FWBUF";

// ============ AT+FWMEMINFO? ============

static uint8_t at_query_cmd_fwmeminfo(uint8_t *cmd_name)
{
    char resp[128];
    
    // Query PSRAM (external SPI RAM)
    size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    size_t psram_largest = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
    
    // Query internal RAM
    size_t internal_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    size_t internal_largest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    
    // Log to console for debugging
    ESP_LOGI(TAG, "PSRAM - Free: %zu bytes, Largest block: %zu bytes", psram_free, psram_largest);
    ESP_LOGI(TAG, "Internal - Free: %zu bytes, Largest block: %zu bytes", internal_free, internal_largest);
    
    // Build response string
    int len = snprintf(resp, sizeof(resp),
		 "+FWMEMINFO:PSRAM,%u,%u\r\n"
		 "+FWMEMINFO:INTERNAL,%u,%u\r\n",
		 (unsigned int)psram_free, (unsigned int)psram_largest, 
		 (unsigned int)internal_free, (unsigned int)internal_largest);
    
    // Send response
    esp_at_port_write_data((uint8_t*)resp, len);
    
    return ESP_AT_RESULT_CODE_OK;
}

// ============ Command Table ============

static const esp_at_cmd_struct at_fw_buffer_cmd[] = {
    {"+FWMEMINFO", NULL, at_query_cmd_fwmeminfo, NULL, NULL},
};

// ============ Registration Function ============
// NOTE: Function name follows ESP-AT convention: esp_at_<module>_cmd_regist

bool esp_at_fw_buffer_cmd_regist(void)
{
    ESP_LOGI(TAG, "Registering firmware buffer AT commands");
    
    return esp_at_custom_cmd_array_regist(
        at_fw_buffer_cmd, 
        sizeof(at_fw_buffer_cmd) / sizeof(at_fw_buffer_cmd[0])
    );
}

