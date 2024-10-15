project(common)

aux_source_directory(connect src)
aux_source_directory(message_center src)

add_library(common STATIC ${src})
