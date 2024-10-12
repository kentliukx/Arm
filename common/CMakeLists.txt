project(common)

aux_source_directory(connect src)

add_library(common STATIC ${src})
