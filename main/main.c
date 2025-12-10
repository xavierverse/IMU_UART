#include <stdio.h>
#include <string.h>
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log_buffer.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
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
#define TIMEOUT_US 200000

#define ICM_ADDRESS 0x68
#define START_REG 0x0B
#define PWR_MGMT_ADDR 0x1F
#define LOW_NOISE_MODE 0x0F

static const char* TAG = "ICM42670";

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle;
static uint8_t packet[25];
static uint8_t data_rd[DATA_LEN];
static uint8_t start_reg = START_REG;


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
        .scl_speed_hz = 400000,
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

uint16_t crc16_ccitt(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < length; i++) {
        crc ^= ((uint16_t)data[i] << 8);

        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}


void send_uart(uint8_t *buf) {
    packet[0] = 0xAA;
    packet[1] = 0x55;


    int64_t time_since_boot = esp_timer_get_time();
    // ESP_LOGI(TAG, "Periodic timer called, time since boot: %lld us", time_since_boot);
    packet[2] = time_since_boot & 0xFF;
    packet[3] = (time_since_boot >> 8) & 0xFF;
    packet[4] = (time_since_boot >> 16) & 0xFF;
    packet[5] = (time_since_boot >> 24) & 0xFF;
    packet[6] = (time_since_boot >> 32) & 0xFF;
    packet[7] = (time_since_boot >> 40) & 0xFF;
    packet[8] = (time_since_boot >> 48) & 0xFF;
    packet[9] = (time_since_boot >> 56) & 0xFF;

    // uint64_t timestamp = ((uint64_t)packet[2])       |
    // ((uint64_t)packet[3] << 8)  |
    // ((uint64_t)packet[4] << 16) |
    // ((uint64_t)packet[5] << 24) |
    // ((uint64_t)packet[6] << 32) |
    // ((uint64_t)packet[7] << 40) |
    // ((uint64_t)packet[8] << 48) |
    // ((uint64_t)packet[9] << 56);
    // ESP_LOGI(TAG, "Timestamp: %llu us", timestamp);

    // Sensor data
    memcpy(&packet[10], buf, 12);

    int16_t accel_x = (int16_t)((packet[10] << 8) | packet[11]);
    // int16_t accel_y = (int16_t)((packet[12] << 8) | packet[13]);
    // int16_t accel_z = (int16_t)((packet[14] << 8) | packet[15]);
    ESP_LOGI(TAG, "Accel XYZ: %d", accel_x);


    uint16_t crc = crc16_ccitt(packet, 22);
    packet[22] = crc & 0xFF;
    packet[23] = (crc >> 8) & 0xFF;

    packet[24] = 0x00;

    uart_write_bytes(UART_PORT_NUM, packet, sizeof(packet));
}

void periodic_timer_callback(void* arg)
{
    uint8_t data_rd[12];
    int ret = i2c_master_transmit_receive(dev_handle, &start_reg, 1, data_rd, 12, 50);
    if (ret == ESP_OK) {
        send_uart(data_rd);
    } else {
        ESP_LOGW(TAG, "I2C read failed: %d", ret);
    }
}


void read_imu(void *arg) {
    // ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    // ESP_ERROR_CHECK(esp_task_wdt_status(NULL));
    // uint8_t start_reg = START_REG;
    // int attempts = 0;
    // esp_err_t ret;

    // while (1) {
    //     esp_task_wdt_reset();
    //     attempts = 0;
    //     while (attempts < RETRY_MAX) {
        
    //         ret = i2c_master_transmit_receive(dev_handle, &start_reg, 1, data_rd, DATA_LEN, 50);
    //         if (ret == ESP_OK) break;

    //         ESP_LOGW(TAG, "I2C read failed: %X", ret);
    //         esp_task_wdt_reset();
    //         vTaskDelay(pdMS_TO_TICKS(10));
    //         attempts++;
    //     }

    //     if (ret != ESP_OK) {
    //         ESP_LOGE(TAG, "Failed to read IMU");
    //     }

    //     send_uart(data_rd);
    //     vTaskDelay(pdMS_TO_TICKS(100));
    
    // }
}

void watchdog_init(void) {
    #if !CONFIG_ESP_TASK_WDT_INIT
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = TWDT_TIMEOUT_S * 1000,
        .idle_core_mask = (1 << CONFIG_FREERTOS_NUMBER_OF_CORES) - 1, // Watch idle tasks on all cores
        .trigger_panic = true, // Trigger a panic on watchdog timeout
    };
    ESP_ERROR_CHECK(esp_task_wdt_init(&twdt_config));
    ESP_LOGI(TAG, "TWDT initialized with timeout of %d seconds.", TWDT_TIMEOUT_S);
    #endif // CONFIG_ESP_TASK_WDT_INIT
}
void timer_init(void) {
    const esp_timer_create_args_t periodic_timer_args = {
            .callback = &periodic_timer_callback,
            .name = "periodic"
    };
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, TIMEOUT_US));
    ESP_LOGI(TAG, "Started timer, time since boot: %lld us", esp_timer_get_time());
}

void app_main(void) {
    icm_init();
    uart_init();
    watchdog_init();
    timer_init();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    // xTaskCreate(read_imu, "read_imu", 4096, NULL, 5, NULL);
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