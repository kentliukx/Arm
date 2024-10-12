project(base)

aux_source_directory(servo src)
aux_source_directory(bsp_buzzer src)
aux_source_directory(tof src)

add_library(base STATIC ${src})
