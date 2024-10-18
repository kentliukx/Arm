project(common)

aux_source_directory(connect src)
aux_source_directory(message_center src)
aux_source_directory(log src)
aux_source_directory(cap_comm src)
aux_source_directory(referee_comm src)
aux_source_directory(board_comm src)
aux_source_directory(bool_input src)

add_library(common STATIC ${src})
