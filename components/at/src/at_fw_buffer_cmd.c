/*
 * Custom AT Commands for Firmware Buffer OTA - Phase 2
 * 
 * This file implements the complete OTA buffer system for downloading
 * firmware to PSRAM and serving it to the S3 master chip.
 * 
 * Commands implemented:
 *   AT+FWMEMINFO?              - Query available PSRAM and internal RAM
 *   AT+FWBUFDOWNLOAD="<url>"   - Download firmware from HTTP URL to PSRAM
 *   AT+FWBUFSTATUS?            - Query buffer state (IDLE/DOWNLOADING/READY/ERROR)
 *   AT+FWBUFREAD=<offset>,<len> - Read chunk from buffer
 *   AT+FWBUFVERIFY=<crc32>     - Verify buffer CRC32
 *   AT+FWBUFCLEAR              - Free PSRAM buffer and reset state
 * 
 * State Machine:
 *   IDLE -> DOWNLOADING -> READY -> IDLE (via CLEAR)
 *                      \-> ERROR -> IDLE (via CLEAR)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_at.h"
#include "esp_at_cmd_register.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_http_client.h"
#include "esp_crc.h"

static const char *TAG = "AT_FWBUF";

// ============ Configuration ============

#define FWBUF_MAX_URL_LEN       256
#define FWBUF_MAX_FIRMWARE_SIZE (2 * 1024 * 1024)  // 2MB max
#define FWBUF_HTTP_TIMEOUT_MS   30000              // 30 second timeout
#define FWBUF_CHUNK_SIZE        4096               // Download chunk size
#define FWBUF_PROTOCOL_VERSION  1

// ============ State Definitions ============

typedef enum {
    FWBUF_STATE_IDLE = 0,
    FWBUF_STATE_DOWNLOADING,
    FWBUF_STATE_READY,
    FWBUF_STATE_ERROR
} fwbuf_state_t;

typedef enum {
    FWBUF_ERR_NONE = 0,
    FWBUF_ERR_NO_MEMORY,
    FWBUF_ERR_HTTP_CONNECT,
    FWBUF_ERR_HTTP_STATUS,
    FWBUF_ERR_HTTP_READ,
    FWBUF_ERR_TOO_LARGE,
    FWBUF_ERR_INVALID_URL,
    FWBUF_ERR_BUSY
} fwbuf_error_t;

// ============ Buffer State ============

typedef struct {
    fwbuf_state_t state;
    fwbuf_error_t last_error;
    uint8_t *buffer;
    uint32_t size;
    uint32_t downloaded;
    uint32_t crc32;
    SemaphoreHandle_t mutex;
} fwbuf_context_t;

static fwbuf_context_t s_fwbuf = {
    .state = FWBUF_STATE_IDLE,
    .last_error = FWBUF_ERR_NONE,
    .buffer = NULL,
    .size = 0,
    .downloaded = 0,
    .crc32 = 0,
    .mutex = NULL
};

// ============ Helper Macros ============

#define FWBUF_LOCK()    xSemaphoreTake(s_fwbuf.mutex, portMAX_DELAY)
#define FWBUF_UNLOCK()  xSemaphoreGive(s_fwbuf.mutex)

// ============ Helper Functions ============

static const char* fwbuf_state_str(fwbuf_state_t state) {
    switch (state) {
        case FWBUF_STATE_IDLE:        return "IDLE";
        case FWBUF_STATE_DOWNLOADING: return "DOWNLOADING";
        case FWBUF_STATE_READY:       return "READY";
        case FWBUF_STATE_ERROR:       return "ERROR";
        default:                      return "UNKNOWN";
    }
}

static void fwbuf_free(void) {
    if (s_fwbuf.buffer) {
        heap_caps_free(s_fwbuf.buffer);
        s_fwbuf.buffer = NULL;
    }
    s_fwbuf.size = 0;
    s_fwbuf.downloaded = 0;
    s_fwbuf.crc32 = 0;
}

// ============ AT+FWMEMINFO? ============
// Query available PSRAM and internal RAM

static uint8_t at_query_cmd_fwmeminfo(uint8_t *cmd_name)
{
    char resp[128];
    
    size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    size_t psram_largest = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
    size_t internal_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    size_t internal_largest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    
    ESP_LOGI(TAG, "PSRAM - Free: %u, Largest: %u", 
             (unsigned int)psram_free, (unsigned int)psram_largest);
    ESP_LOGI(TAG, "Internal - Free: %u, Largest: %u", 
             (unsigned int)internal_free, (unsigned int)internal_largest);
    
    int len = snprintf(resp, sizeof(resp),
             "+FWMEMINFO:PSRAM,%u,%u\r\n"
             "+FWMEMINFO:INTERNAL,%u,%u\r\n",
             (unsigned int)psram_free, (unsigned int)psram_largest, 
             (unsigned int)internal_free, (unsigned int)internal_largest);
    
    esp_at_port_write_data((uint8_t*)resp, len);
    
    return ESP_AT_RESULT_CODE_OK;
}

// ============ AT+FWBUFSTATUS? ============
// Query buffer state, size, and CRC

static uint8_t at_query_cmd_fwbufstatus(uint8_t *cmd_name)
{
    char resp[128];
    
    FWBUF_LOCK();
    
    int len;
    switch (s_fwbuf.state) {
        case FWBUF_STATE_IDLE:
            len = snprintf(resp, sizeof(resp),
                     "+FWBUFSTATUS:%d,IDLE,0,0\r\n",
                     FWBUF_PROTOCOL_VERSION);
            break;
            
        case FWBUF_STATE_DOWNLOADING:
            len = snprintf(resp, sizeof(resp),
                     "+FWBUFSTATUS:%d,DOWNLOADING,%u,%u\r\n",
                     FWBUF_PROTOCOL_VERSION,
                     (unsigned int)s_fwbuf.downloaded,
                     (unsigned int)s_fwbuf.size);
            break;
            
        case FWBUF_STATE_READY:
            len = snprintf(resp, sizeof(resp),
                     "+FWBUFSTATUS:%d,READY,%u,0x%08X\r\n",
                     FWBUF_PROTOCOL_VERSION,
                     (unsigned int)s_fwbuf.size,
                     (unsigned int)s_fwbuf.crc32);
            break;
            
        case FWBUF_STATE_ERROR:
            len = snprintf(resp, sizeof(resp),
                     "+FWBUFSTATUS:%d,ERROR,%d,0\r\n",
                     FWBUF_PROTOCOL_VERSION,
                     s_fwbuf.last_error);
            break;
            
        default:
            len = snprintf(resp, sizeof(resp),
                     "+FWBUFSTATUS:%d,UNKNOWN,0,0\r\n",
                     FWBUF_PROTOCOL_VERSION);
            break;
    }
    
    FWBUF_UNLOCK();
    
    esp_at_port_write_data((uint8_t*)resp, len);
    
    return ESP_AT_RESULT_CODE_OK;
}

// ============ AT+FWBUFDOWNLOAD="<url>" ============
// Download firmware from HTTP URL to PSRAM buffer

static uint8_t at_setup_cmd_fwbufdownload(uint8_t para_num)
{
    uint8_t *url = NULL;
    char resp[128];
    int len;
    
    // Parse URL parameter
    if (para_num != 1) {
        return ESP_AT_RESULT_CODE_ERROR;
    }
    
    if (esp_at_get_para_as_str(0, &url) != ESP_AT_PARA_PARSE_RESULT_OK) {
        return ESP_AT_RESULT_CODE_ERROR;
    }
    
    if (url == NULL || strlen((char*)url) == 0) {
        return ESP_AT_RESULT_CODE_ERROR;
    }
    
    // Validate URL format (must start with http://)
    if (strncmp((char*)url, "http://", 7) != 0) {
        ESP_LOGE(TAG, "Invalid URL: must start with http://");
        esp_at_port_write_data((uint8_t*)"+FWBUFDOWNLOAD:ERROR,INVALID_URL\r\n", 34);
        return ESP_AT_RESULT_CODE_ERROR;
    }
    
    if (strlen((char*)url) > FWBUF_MAX_URL_LEN) {
        ESP_LOGE(TAG, "URL too long: max %d chars", FWBUF_MAX_URL_LEN);
        esp_at_port_write_data((uint8_t*)"+FWBUFDOWNLOAD:ERROR,URL_TOO_LONG\r\n", 35);
        return ESP_AT_RESULT_CODE_ERROR;
    }
    
    FWBUF_LOCK();
    
    // Check state
    if (s_fwbuf.state == FWBUF_STATE_DOWNLOADING) {
        FWBUF_UNLOCK();
        esp_at_port_write_data((uint8_t*)"+FWBUFDOWNLOAD:ERROR,BUSY\r\n", 27);
        return ESP_AT_RESULT_CODE_ERROR;
    }
    
    // If we have an existing buffer, free it
    if (s_fwbuf.state == FWBUF_STATE_READY || s_fwbuf.state == FWBUF_STATE_ERROR) {
        fwbuf_free();
    }
    
    // Transition to DOWNLOADING
    s_fwbuf.state = FWBUF_STATE_DOWNLOADING;
    s_fwbuf.last_error = FWBUF_ERR_NONE;
    s_fwbuf.downloaded = 0;
    
    FWBUF_UNLOCK();
    
    // Send STARTED response
    esp_at_port_write_data((uint8_t*)"+FWBUFDOWNLOAD:STARTED\r\n", 24);
    
    ESP_LOGI(TAG, "Starting download from: %s", url);
    
    // Configure HTTP client
    esp_http_client_config_t config = {
        .url = (char*)url,
        .timeout_ms = FWBUF_HTTP_TIMEOUT_MS,
        .buffer_size = FWBUF_CHUNK_SIZE,
    };
    
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        ESP_LOGE(TAG, "Failed to init HTTP client");
        FWBUF_LOCK();
        s_fwbuf.state = FWBUF_STATE_ERROR;
        s_fwbuf.last_error = FWBUF_ERR_HTTP_CONNECT;
        FWBUF_UNLOCK();
        esp_at_port_write_data((uint8_t*)"+FWBUFDOWNLOAD:ERROR,HTTP_INIT\r\n", 32);
        return ESP_AT_RESULT_CODE_ERROR;
    }
    
    // Open connection
    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        FWBUF_LOCK();
        s_fwbuf.state = FWBUF_STATE_ERROR;
        s_fwbuf.last_error = FWBUF_ERR_HTTP_CONNECT;
        FWBUF_UNLOCK();
        len = snprintf(resp, sizeof(resp), "+FWBUFDOWNLOAD:ERROR,HTTP_CONNECT,%d\r\n", err);
        esp_at_port_write_data((uint8_t*)resp, len);
        return ESP_AT_RESULT_CODE_ERROR;
    }
    
    // Fetch headers
    int content_length = esp_http_client_fetch_headers(client);
    int status_code = esp_http_client_get_status_code(client);
    
    ESP_LOGI(TAG, "HTTP status: %d, Content-Length: %d", status_code, content_length);
    
    if (status_code != 200) {
        ESP_LOGE(TAG, "HTTP error: status %d", status_code);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        FWBUF_LOCK();
        s_fwbuf.state = FWBUF_STATE_ERROR;
        s_fwbuf.last_error = FWBUF_ERR_HTTP_STATUS;
        FWBUF_UNLOCK();
        len = snprintf(resp, sizeof(resp), "+FWBUFDOWNLOAD:ERROR,HTTP_STATUS,%d\r\n", status_code);
        esp_at_port_write_data((uint8_t*)resp, len);
        return ESP_AT_RESULT_CODE_ERROR;
    }
    
    // Check content length
    if (content_length <= 0) {
        ESP_LOGW(TAG, "Unknown content length, will read until EOF");
        content_length = FWBUF_MAX_FIRMWARE_SIZE;  // Allocate max and trim later
    }
    
    if (content_length > FWBUF_MAX_FIRMWARE_SIZE) {
        ESP_LOGE(TAG, "Firmware too large: %d > %d", content_length, FWBUF_MAX_FIRMWARE_SIZE);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        FWBUF_LOCK();
        s_fwbuf.state = FWBUF_STATE_ERROR;
        s_fwbuf.last_error = FWBUF_ERR_TOO_LARGE;
        FWBUF_UNLOCK();
        len = snprintf(resp, sizeof(resp), "+FWBUFDOWNLOAD:ERROR,TOO_LARGE,%d\r\n", content_length);
        esp_at_port_write_data((uint8_t*)resp, len);
        return ESP_AT_RESULT_CODE_ERROR;
    }
    
    // Allocate PSRAM buffer
    FWBUF_LOCK();
    s_fwbuf.buffer = heap_caps_malloc(content_length, MALLOC_CAP_SPIRAM);
    if (s_fwbuf.buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate %d bytes in PSRAM", content_length);
        FWBUF_UNLOCK();
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        FWBUF_LOCK();
        s_fwbuf.state = FWBUF_STATE_ERROR;
        s_fwbuf.last_error = FWBUF_ERR_NO_MEMORY;
        FWBUF_UNLOCK();
        esp_at_port_write_data((uint8_t*)"+FWBUFDOWNLOAD:ERROR,NO_MEMORY\r\n", 32);
        return ESP_AT_RESULT_CODE_ERROR;
    }
    s_fwbuf.size = content_length;
    FWBUF_UNLOCK();
    
    ESP_LOGI(TAG, "Allocated %d bytes in PSRAM", content_length);
    
    // Read data
    int total_read = 0;
    int read_len;
    uint32_t crc = 0;
    
    while (total_read < content_length) {
        read_len = esp_http_client_read(client, (char*)(s_fwbuf.buffer + total_read), 
                                        FWBUF_CHUNK_SIZE);
        
        if (read_len < 0) {
            ESP_LOGE(TAG, "HTTP read error");
            esp_http_client_close(client);
            esp_http_client_cleanup(client);
            FWBUF_LOCK();
            fwbuf_free();
            s_fwbuf.state = FWBUF_STATE_ERROR;
            s_fwbuf.last_error = FWBUF_ERR_HTTP_READ;
            FWBUF_UNLOCK();
            esp_at_port_write_data((uint8_t*)"+FWBUFDOWNLOAD:ERROR,HTTP_READ\r\n", 32);
            return ESP_AT_RESULT_CODE_ERROR;
        }
        
        if (read_len == 0) {
            // EOF reached
            ESP_LOGI(TAG, "EOF reached after %d bytes", total_read);
            break;
        }
        
        // Update CRC
        crc = esp_crc32_le(crc, s_fwbuf.buffer + total_read, read_len);
        
        total_read += read_len;
        
        FWBUF_LOCK();
        s_fwbuf.downloaded = total_read;
        FWBUF_UNLOCK();
        
        // Log progress every 100KB
        if ((total_read % (100 * 1024)) < FWBUF_CHUNK_SIZE) {
            ESP_LOGI(TAG, "Downloaded: %d / %d bytes (%.1f%%)", 
                     total_read, content_length, 
                     (float)total_read * 100.0f / content_length);
        }
    }
    
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    
    // Update final state
    FWBUF_LOCK();
    s_fwbuf.size = total_read;  // Update to actual size
    s_fwbuf.downloaded = total_read;
    s_fwbuf.crc32 = crc;
    s_fwbuf.state = FWBUF_STATE_READY;
    FWBUF_UNLOCK();
    
    ESP_LOGI(TAG, "Download complete: %d bytes, CRC32: 0x%08X", total_read, (unsigned int)crc);
    
    // Send completion response
    len = snprintf(resp, sizeof(resp), "+FWBUFDOWNLOAD:DONE,%u,0x%08X\r\n",
                   (unsigned int)total_read, (unsigned int)crc);
    esp_at_port_write_data((uint8_t*)resp, len);
    
    return ESP_AT_RESULT_CODE_OK;
}

// ============ AT+FWBUFREAD=<offset>,<length> ============
// Read chunk from buffer with length-prefixed binary framing

static uint8_t at_setup_cmd_fwbufread(uint8_t para_num)
{
    int32_t offset = 0;
    int32_t length = 0;
    char header[64];
    int header_len;
    
    // Parse parameters
    if (para_num != 2) {
        return ESP_AT_RESULT_CODE_ERROR;
    }
    
    if (esp_at_get_para_as_digit(0, &offset) != ESP_AT_PARA_PARSE_RESULT_OK) {
        return ESP_AT_RESULT_CODE_ERROR;
    }
    
    if (esp_at_get_para_as_digit(1, &length) != ESP_AT_PARA_PARSE_RESULT_OK) {
        return ESP_AT_RESULT_CODE_ERROR;
    }
    
    // Validate parameters
    if (offset < 0 || length <= 0 || length > 4096) {
        ESP_LOGE(TAG, "Invalid params: offset=%d, length=%d", (int)offset, (int)length);
        esp_at_port_write_data((uint8_t*)"+FWBUFREAD:ERROR,INVALID_PARAMS\r\n", 33);
        return ESP_AT_RESULT_CODE_ERROR;
    }
    
    FWBUF_LOCK();
    
    // Check state
    if (s_fwbuf.state != FWBUF_STATE_READY) {
        FWBUF_UNLOCK();
        header_len = snprintf(header, sizeof(header), "+FWBUFREAD:ERROR,%s\r\n", 
                              fwbuf_state_str(s_fwbuf.state));
        esp_at_port_write_data((uint8_t*)header, header_len);
        return ESP_AT_RESULT_CODE_ERROR;
    }
    
    // Validate offset
    if (offset >= (int32_t)s_fwbuf.size) {
        FWBUF_UNLOCK();
        esp_at_port_write_data((uint8_t*)"+FWBUFREAD:ERROR,OFFSET_OUT_OF_RANGE\r\n", 38);
        return ESP_AT_RESULT_CODE_ERROR;
    }
    
    // Clamp length to available data
    int32_t available = s_fwbuf.size - offset;
    if (length > available) {
        length = available;
    }
    
    // Send response with length-prefixed binary data
    // Format: +FWBUFREAD:<actual_length>,<binary_data>\r\n\r\nOK\r\n
    header_len = snprintf(header, sizeof(header), "+FWBUFREAD:%d,", (int)length);
    esp_at_port_write_data((uint8_t*)header, header_len);
    
    // Send binary data
    esp_at_port_write_data(s_fwbuf.buffer + offset, length);
    
    FWBUF_UNLOCK();
    
    // Send trailer
    esp_at_port_write_data((uint8_t*)"\r\n", 2);
    
    return ESP_AT_RESULT_CODE_OK;
}

// ============ AT+FWBUFVERIFY=<expected_crc32> ============
// Verify buffer CRC32 matches expected value

static uint8_t at_setup_cmd_fwbufverify(uint8_t para_num)
{
    int32_t expected_crc = 0;
    char resp[64];
    int len;
    
    // Parse parameter (hex or decimal)
    if (para_num != 1) {
        return ESP_AT_RESULT_CODE_ERROR;
    }
    
    // Get as string first to handle hex format
    uint8_t *crc_str = NULL;
    if (esp_at_get_para_as_str(0, &crc_str) == ESP_AT_PARA_PARSE_RESULT_OK && crc_str != NULL) {
        // Try to parse as hex (0x prefix) or decimal
        if (strncmp((char*)crc_str, "0x", 2) == 0 || strncmp((char*)crc_str, "0X", 2) == 0) {
            expected_crc = strtoul((char*)crc_str + 2, NULL, 16);
        } else {
            expected_crc = strtoul((char*)crc_str, NULL, 10);
        }
    } else {
        // Try as digit
        if (esp_at_get_para_as_digit(0, &expected_crc) != ESP_AT_PARA_PARSE_RESULT_OK) {
            return ESP_AT_RESULT_CODE_ERROR;
        }
    }
    
    FWBUF_LOCK();
    
    // Check state
    if (s_fwbuf.state != FWBUF_STATE_READY) {
        FWBUF_UNLOCK();
        len = snprintf(resp, sizeof(resp), "+FWBUFVERIFY:ERROR,%s\r\n", 
                       fwbuf_state_str(s_fwbuf.state));
        esp_at_port_write_data((uint8_t*)resp, len);
        return ESP_AT_RESULT_CODE_ERROR;
    }
    
    // Compare CRC
    uint32_t actual_crc = s_fwbuf.crc32;
    FWBUF_UNLOCK();
    
    if ((uint32_t)expected_crc == actual_crc) {
        ESP_LOGI(TAG, "CRC32 verified: 0x%08X", (unsigned int)actual_crc);
        len = snprintf(resp, sizeof(resp), "+FWBUFVERIFY:OK,0x%08X\r\n", (unsigned int)actual_crc);
        esp_at_port_write_data((uint8_t*)resp, len);
        return ESP_AT_RESULT_CODE_OK;
    } else {
        ESP_LOGW(TAG, "CRC32 mismatch: expected 0x%08X, actual 0x%08X", 
                 (unsigned int)expected_crc, (unsigned int)actual_crc);
        len = snprintf(resp, sizeof(resp), "+FWBUFVERIFY:MISMATCH,0x%08X\r\n", (unsigned int)actual_crc);
        esp_at_port_write_data((uint8_t*)resp, len);
        return ESP_AT_RESULT_CODE_ERROR;
    }
}

// ============ AT+FWBUFCLEAR ============
// Free PSRAM buffer and reset state to IDLE

static uint8_t at_exe_cmd_fwbufclear(uint8_t *cmd_name)
{
    FWBUF_LOCK();
    
    // Don't allow clear while downloading
    if (s_fwbuf.state == FWBUF_STATE_DOWNLOADING) {
        FWBUF_UNLOCK();
        esp_at_port_write_data((uint8_t*)"+FWBUFCLEAR:ERROR,BUSY\r\n", 24);
        return ESP_AT_RESULT_CODE_ERROR;
    }
    
    // Free buffer and reset state
    fwbuf_free();
    s_fwbuf.state = FWBUF_STATE_IDLE;
    s_fwbuf.last_error = FWBUF_ERR_NONE;
    
    FWBUF_UNLOCK();
    
    ESP_LOGI(TAG, "Buffer cleared");
    
    return ESP_AT_RESULT_CODE_OK;
}

// ============ Command Table ============
// Format: {name, TEST, QUERY, SETUP, EXECUTE}

static const esp_at_cmd_struct at_fw_buffer_cmd[] = {
    {"+FWMEMINFO",     NULL, at_query_cmd_fwmeminfo,   NULL,                      NULL},
    {"+FWBUFSTATUS",   NULL, at_query_cmd_fwbufstatus, NULL,                      NULL},
    {"+FWBUFDOWNLOAD", NULL, NULL,                     at_setup_cmd_fwbufdownload, NULL},
    {"+FWBUFREAD",     NULL, NULL,                     at_setup_cmd_fwbufread,    NULL},
    {"+FWBUFVERIFY",   NULL, NULL,                     at_setup_cmd_fwbufverify,  NULL},
    {"+FWBUFCLEAR",    NULL, NULL,                     NULL,                      at_exe_cmd_fwbufclear},
};

// ============ Registration Function ============

bool esp_at_fw_buffer_cmd_regist(void)
{
    ESP_LOGI(TAG, "Registering firmware buffer AT commands (Phase 2)");
    
    // Create mutex for thread safety
    if (s_fwbuf.mutex == NULL) {
        s_fwbuf.mutex = xSemaphoreCreateMutex();
        if (s_fwbuf.mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create mutex");
            return false;
        }
    }
    
    // Initialize state
    s_fwbuf.state = FWBUF_STATE_IDLE;
    s_fwbuf.last_error = FWBUF_ERR_NONE;
    s_fwbuf.buffer = NULL;
    s_fwbuf.size = 0;
    s_fwbuf.downloaded = 0;
    s_fwbuf.crc32 = 0;
    
    return esp_at_custom_cmd_array_regist(
        at_fw_buffer_cmd, 
        sizeof(at_fw_buffer_cmd) / sizeof(at_fw_buffer_cmd[0])
    );
}
