project(algorithm)

aux_source_directory(pid src)
aux_source_directory(filter src)
aux_source_directory(math src)
aux_source_directory(fifo_buffer src)

add_library(algorithm STATIC ${src})
