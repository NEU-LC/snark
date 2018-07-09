if( snark_build_sensors_basler )
    add_subdirectory( basler )
endif( snark_build_sensors_basler )

IF( snark_build_sensors_dc1394 )
    ADD_SUBDIRECTORY( dc1394 )
ENDIF( snark_build_sensors_dc1394 )

IF( snark_build_sensors_gige )
    ADD_SUBDIRECTORY( gige )
ENDIF( snark_build_sensors_gige )

IF( snark_build_sensors_flycapture )
    ADD_SUBDIRECTORY( flycapture )
ENDIF( snark_build_sensors_flycapture )

IF( snark_build_sensors_gobi )
    ADD_SUBDIRECTORY( gobi )
ENDIF( snark_build_sensors_gobi )

if( snark_build_sensors_jai )
    add_subdirectory( jai )
endif( snark_build_sensors_jai )

if( snark_build_sensors_kinect )
    add_subdirectory( kinect )
endif( snark_build_sensors_kinect )

IF( snark_build_sensors_vimba )
    ADD_SUBDIRECTORY( vimba )
ENDIF( snark_build_sensors_vimba )

if(snark_build_sensors_realsense)
    add_subdirectory(realsense)
endif()

if(snark_build_sensors_realsense2)
    add_subdirectory(realsense2)
endif()
