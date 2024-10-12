project(base)

aux_source_directory(servo src)
aux_source_directory(bsp_buzzer src)

add_library(base STATIC ${src})
