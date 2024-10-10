project(base)

aux_source_directory(chassis src)

add_library(base STATIC ${src})
