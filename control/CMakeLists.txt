SET( dir ${SOURCE_CODE_BASE_DIR}/control )

IF( snark_BUILD_APPLICATIONS )
    ADD_SUBDIRECTORY( applications )
    EXECUTE_PROCESS( COMMAND rm -f ${dir}/test/disabled )
ENDIF( snark_BUILD_APPLICATIONS )
