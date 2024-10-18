project(base)

aux_source_directory(servo src)
aux_source_directory(bsp_buzzer src)
aux_source_directory(encoder src)
aux_source_directory(motor src)
aux_source_directory(motor/motor_driver src)
aux_source_directory(bsp_led src)
aux_source_directory(remote src)
aux_source_directory(tof src)

add_library(base STATIC ${src})
