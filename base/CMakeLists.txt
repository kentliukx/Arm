project(base)

aux_source_directory(servo src)

add_library(base STATIC ${src})
