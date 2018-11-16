add_subdirectory( cameras )
add_subdirectory( lidars )
add_subdirectory( spectrometers )
add_subdirectory( weather )

if( snark_build_sensors_roboteye )
    add_subdirectory( roboteye )
endif()
