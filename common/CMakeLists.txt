project(common)

aux_source_directory(connect src)
aux_source_directory(message_center src)
aux_source_directory(log src)

add_library(common STATIC ${src})
