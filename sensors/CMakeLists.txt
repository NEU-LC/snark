IF( snark_build_sensors_dc1394 )
    ADD_SUBDIRECTORY( dc1394 )
ENDIF( snark_build_sensors_dc1394 )

IF( snark_build_sensors_gige )
    ADD_SUBDIRECTORY( gige )
ENDIF( snark_build_sensors_gige )

IF( snark_build_sensors_flycapture )
    ADD_SUBDIRECTORY( flycapture )
ENDIF( snark_build_sensors_flycapture )

IF( snark_build_sensors_sick )
    ADD_SUBDIRECTORY( sick )
ENDIF( snark_build_sensors_sick )

IF( snark_build_sensors_velodyne )
    ADD_SUBDIRECTORY( velodyne )
ENDIF( snark_build_sensors_velodyne )

IF( snark_build_sensors_hokuyo )
    add_subdirectory(hokuyo)
ENDIF( snark_build_sensors_hokuyo )

IF( snark_build_sensors_gobi )
    ADD_SUBDIRECTORY( gobi )
ENDIF( snark_build_sensors_gobi )

add_subdirectory( las )
