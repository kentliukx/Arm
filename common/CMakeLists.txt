project(common)

aux_source_directory(connect src)
aux_source_directory(message_center src)
aux_source_directory(log src)
aux_source_directory(communication/board_comm src)
aux_source_directory(bool_input src)
aux_source_directory(power_limit src)
#aux_source_directory(heat_limit src)
aux_source_directory(fifo src)
add_subdirectory(communication)

add_library(common STATIC ${src})
