idf_component_register(SRCS "led_control.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver esp_common esp_driver_gpio)
