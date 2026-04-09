idf_component_register(SRCS "position_control.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver esp_timer freertos log bdc_motor_driver)
