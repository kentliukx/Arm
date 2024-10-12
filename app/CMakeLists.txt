project(app)

aux_source_directory(chassis src)

add_library(app STATIC ${src})
