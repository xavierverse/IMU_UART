# IMU_UART
Project reading from IMU on ESP32C3 that sends data over UART at 100 Hz.

# Issues 
1. Task Watchdog. I2C_ll_is_busy
    Fix 1: Use xTaskCreate to create read task, Use static buffers to store accel and gyro data across calls (stack overflow was occurring with stack allocated buffers inside of read_imu function)
    Fix 2: Use espidf periodic timer to perform reads and pass raw data to uart buffer
2. IMU read fails / i2c clear bus failed / unexpected NACK
    Fixes: feed watchdog and reset attempts after each loop and on i2c read fail
3. Float & int conversion issues when sending data over uart
    Fixes: Increase buffer size to allow for full sensor data. 2 bytes -> 4 bytes for accel/gyro XYZ
4. Speed up i2C reads and uart
    Fixes: Send raw data over uart, expensive to convert to float
