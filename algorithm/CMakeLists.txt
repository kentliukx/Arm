project(lib)

aux_source_directory(pid src)

add_library(lib STATIC ${src})
