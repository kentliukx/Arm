project(middleware)

aux_source_directory(SEGGER/Config src)
aux_source_directory(SEGGER/RTT src)

add_library(middleware STATIC ${src})
