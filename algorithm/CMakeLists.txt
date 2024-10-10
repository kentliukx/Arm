project(algorithm)

aux_source_directory(pid src)
aux_source_directory(filter src)
aux_source_directory(math src)

add_library(algorithm STATIC ${src})
