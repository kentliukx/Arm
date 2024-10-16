project(driver)

aux_source_directory(motor src)
aux_source_directory(motor/motor_driver src)

add_library(driver STATIC ${src})
