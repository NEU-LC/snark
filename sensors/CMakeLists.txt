add_subdirectory( cameras )
add_subdirectory( lidars )

if( snark_build_sensors_asd )
    add_subdirectory(asd)
endif( snark_build_sensors_asd )
