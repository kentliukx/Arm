project(app)

aux_source_directory(chassis src)
aux_source_directory(control src)
aux_source_directory(communication src)

add_library(app STATIC ${src})
