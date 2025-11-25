idf_component_register(SRCS "control_serial.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES esp_driver_gpio esp_driver_uart stepper_motor)
