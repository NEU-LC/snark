IF( snark_build_sensors_dc1394 )
    ADD_SUBDIRECTORY( dc1394 )
ENDIF( snark_build_sensors_dc1394 )

IF( snark_build_sensors_gige )
    ADD_SUBDIRECTORY( gige )
ENDIF( snark_build_sensors_gige )

IF( snark_build_sensors_sick )
    ADD_SUBDIRECTORY( sick )
ENDIF( snark_build_sensors_sick )

IF( snark_build_sensors_velodyne )
    ADD_SUBDIRECTORY( velodyne )
ENDIF( snark_build_sensors_velodyne )
