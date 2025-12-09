# IMU_UART
Project reading from IMU on ESP32C3 that sends data over UART at 100 Hz.

# Issues 
1. Task Watchdog. I2C_ll_is_busy
    Fixes: Use xTaskCreate to create read task, Use static buffers to store accel and gyro data across calls (stack overflow was occurring with stack allocated buffers inside of read_imu function)
2. IMU read fails / i2c clear bus failed / unexpected NACK
    Fixes: feed watchdog and reset attempts after each loop and on i2c read fail
