SET( PROJECT "universal_robots" )
SET( dir ${snark_actuators_dir}/${PROJECT} )

IF( snark_BUILD_APPLICATIONS )
    ADD_SUBDIRECTORY( applications )
    EXECUTE_PROCESS( COMMAND rm -f ${dir}/test/disabled )
ENDIF( snark_BUILD_APPLICATIONS )

