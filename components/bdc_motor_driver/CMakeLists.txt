idf_component_register(SRCS "bdc_motor_driver.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver esp_timer freertos log)
