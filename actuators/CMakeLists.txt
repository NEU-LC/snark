IF( snark_build_actuators_quickset )
	ADD_SUBDIRECTORY( quickset )
ENDIF( snark_build_actuators_quickset )

IF( snark_build_actuators_wheels )
	ADD_SUBDIRECTORY( wheels )
ENDIF( snark_build_actuators_wheels )

IF( snark_build_actuators_robotic_arm )
	ADD_SUBDIRECTORY( universal_robots )
ENDIF( snark_build_actuators_robotic_arm )
