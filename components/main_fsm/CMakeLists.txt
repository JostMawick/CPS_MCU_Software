idf_component_register(SRCS "main_fsm.c"
                    INCLUDE_DIRS "include"
                    REQUIRES digital_input position_control led_control servo_control)
