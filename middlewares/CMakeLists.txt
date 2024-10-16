project(middlewares)

aux_source_directory(SEGGER/Config src)
aux_source_directory(SEGGER/RTT src)

add_library(middlewares STATIC ${src})
