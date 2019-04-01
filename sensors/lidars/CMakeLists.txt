add_subdirectory( las )

IF( snark_build_sensors_sick )
    ADD_SUBDIRECTORY( sick )
ENDIF( snark_build_sensors_sick )

IF( snark_build_sensors_velodyne )
    ADD_SUBDIRECTORY( velodyne )
ENDIF( snark_build_sensors_velodyne )

IF( snark_build_sensors_hokuyo )
    add_subdirectory(hokuyo)
ENDIF( snark_build_sensors_hokuyo )

if(snark_build_sensors_cepton)
    add_subdirectory(cepton)
endif()

add_subdirectory( robosense )
