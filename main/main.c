#include <stdio.h>
#include <string.h>
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log_buffer.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "esp_task_wdt.h"
#define I2C_MASTER_SCL_IO 8
#define I2C_MASTER_SDA_IO 10
#define DATA_LEN 12
#define RETRY_MAX 5
#define TWDT_TIMEOUT_S 5
#define UART_BAUD_RATE 115200
#define UART_PORT_NUM UART_NUM_0
#define BUF_SIZE (1024)
#define TXD  GPIO_NUM_21
#define RXD GPIO_NUM_20
#define RTS
#define CTS

#define ICM_ADDRESS 0x68
#define START_REG 0x0B
#define PWR_MGMT_ADDR 0x1F
#define LOW_NOISE_MODE 0x0F

static const char* TAG = "ICM42670";

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle;
static float accel[3], gyro[3];


esp_err_t icm_init(void) {
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = 0,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ICM_ADDRESS,
        .scl_speed_hz = 100000,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

    uint8_t wake_buf[2] = { PWR_MGMT_ADDR, LOW_NOISE_MODE } ;

    // Wake up cmd
    i2c_master_transmit(dev_handle, wake_buf, sizeof(wake_buf), -1);
    vTaskDelay(pdMS_TO_TICKS(50));

    ESP_LOGI(TAG, "I2C master + device initialized.");
    return ESP_OK;
}

void uart_init(void) {
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, TXD, RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
}

void send_uart(float *accel, float *gyro) {
    uart_write_bytes(UART_PORT_NUM, )
}


void read_imu(void *arg) {
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    ESP_ERROR_CHECK(esp_task_wdt_status(NULL));
    uint8_t start_reg = START_REG;
    uint8_t data_rd[DATA_LEN];
    int attempts = 0;
    esp_err_t ret;

    while (1) {
        esp_task_wdt_reset();
        attempts = 0;
        while (attempts < RETRY_MAX) {
        
            ret = i2c_master_transmit_receive(dev_handle, &start_reg, 1, data_rd, DATA_LEN, 50);
            if (ret == ESP_OK) break;

            ESP_LOGW(TAG, "I2C read failed: %X", ret);
            esp_task_wdt_reset();
            vTaskDelay(pdMS_TO_TICKS(10));
            attempts++;
        }

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read IMU");
        }

        // Read sensor data
        int16_t accel_x = (int16_t)((data_rd[0] << 8) | data_rd[1]);
        int16_t accel_y = (int16_t)((data_rd[2] << 8) | data_rd[3]);
        int16_t accel_z = (int16_t)((data_rd[4] << 8) | data_rd[5]);

        int16_t gyro_x = (int16_t)((data_rd[6] << 8) | data_rd[7]);
        int16_t gyro_y = (int16_t)((data_rd[8] << 8) | data_rd[9]);
        int16_t gyro_z = (int16_t)((data_rd[10] << 8) | data_rd[11]);

        // Convert to float based on sensitivity
        accel[0] = (float)accel_x / 2048.0f;
        accel[1] = (float)accel_y / 2048.0f;
        accel[2] = (float)accel_z / 2048.0f;

        gyro[0] = (float)gyro_x / 16.4f;
        gyro[1] = (float)gyro_y / 16.4f;
        gyro[2] = (float)gyro_z / 16.4f;

        ESP_LOGI(TAG, "Accel XYZ: %.4f %.4f %.4f Gyro XYZ: %.4f %.4f %.4f", accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2]);
        vTaskDelay(pdMS_TO_TICKS(200));
    
    }
}

void app_main(void) {
    icm_init();
    uart_init();
    #if !CONFIG_ESP_TASK_WDT_INIT
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = TWDT_TIMEOUT_S * 1000,
        .idle_core_mask = (1 << CONFIG_FREERTOS_NUMBER_OF_CORES) - 1, // Watch idle tasks on all cores
        .trigger_panic = true, // Trigger a panic on watchdog timeout
    };
    ESP_ERROR_CHECK(esp_task_wdt_init(&twdt_config));
    ESP_LOGI(TAG, "TWDT initialized with timeout of %d seconds.", TWDT_TIMEOUT_S);
    #endif // CONFIG_ESP_TASK_WDT_INIT

    xTaskCreate(read_imu, "read_imu", 4096, NULL, 5, NULL);
}

void calculate_sensitivity(uint8_t gyro_fs_sel, uint8_t accel_fs_sel) {
    uint8_t gyro_cfg = 0x20, accel_cfg = 0x21;
    uint8_t gyro_sens, accel_sens;

    i2c_master_transmit(dev_handle, &gyro_cfg, 1, -1);
    i2c_master_receive(dev_handle, &gyro_sens, 1, -1);
    gyro_fs_sel = (gyro_sens >> 5) & 0x07;

    i2c_master_transmit(dev_handle, &accel_cfg, 1, -1);
    i2c_master_receive(dev_handle, &accel_sens, 1, -1);
    accel_fs_sel = (accel_sens >> 5) & 0x07;


    ESP_LOGI(TAG, "Gyro Sens: %d, Accel Sens: %d", gyro_fs_sel, accel_fs_sel);
}