# IMU_UART
Project reading from IMU on ESP32C3 that sends data over UART at 100 Hz.

# Issues 
1. Task Watchdog. I2C_ll_is_busy
2. If IMU read fails, reset safely
