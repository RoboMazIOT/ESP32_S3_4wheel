idf_component_register(
    SRCS
        "main.cpp"
        "Motor/MecanumRobot.cpp"
        "PWM/MAZPWM.cpp"
        "LCD/MAZLCD.cpp"
        "Compass/MAZGY271.cpp"
        "IMU/MAZMPU6050.cpp"
    INCLUDE_DIRS
        "."
    REQUIRES
        esp_driver_gpio   # driver/gpio.h
        esp_driver_mcpwm  # driver/mcpwm_*.h — IDF MCPWM driver
        esp_driver_i2c    # driver/i2c_master.h — I2C for LCD
        led_strip         # WS2812B on GPIO48 via RMT
        freertos
        log
)
