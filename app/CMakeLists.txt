project(app)

aux_source_directory(chassis src)
aux_source_directory(control src)
aux_source_directory(gimbal src)
aux_source_directory(motorinit src)
aux_source_directory(shoot src)
aux_source_directory(communication src)

add_library(app STATIC ${src}
        demo_arm/arm.h
        demo_arm/arm.h
        demo_arm/arm.cpp
        demo_arm/rc_to_theta.cpp
)
