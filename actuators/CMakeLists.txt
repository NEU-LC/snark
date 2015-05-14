SET( snark_actuators_dir ${SOURCE_CODE_BASE_DIR}/actuators )

IF( snark_build_actuators_quickset )
	ADD_SUBDIRECTORY( quickset )
ENDIF( snark_build_actuators_quickset )

IF( snark_build_actuators_wheels )
	ADD_SUBDIRECTORY( wheels )
ENDIF( snark_build_actuators_wheels )

IF( snark_build_actuators_universal_robots )
	ADD_SUBDIRECTORY( universal_robots )
ENDIF( snark_build_actuators_universal_robots )
