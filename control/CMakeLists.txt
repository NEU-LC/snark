SET( PROJECT "control" )
IF( NOT snark_build_control )
    EXECUTE_PROCESS( COMMAND touch ${SOURCE_CODE_BASE_DIR}/${PROJECT}/applications/test/disabled )
ELSE( NOT snark_build_control )
    SET( TARGET_NAME snark_${PROJECT} )

    FILE( GLOB source ${SOURCE_CODE_BASE_DIR}/${PROJECT}/*.cpp )
    FILE( GLOB includes ${SOURCE_CODE_BASE_DIR}/${PROJECT}/*.h )

    SOURCE_GROUP( ${TARGET_NAME} FILES ${source} ${includes} )

    ADD_LIBRARY( ${TARGET_NAME} ${source} ${includes} )
    SET_TARGET_PROPERTIES( ${TARGET_NAME} PROPERTIES ${snark_LIBRARY_PROPERTIES} )
    TARGET_LINK_LIBRARIES( ${TARGET_NAME} ${comma_ALL_LIBRARIES} ${comma_ALL_EXTERNAL_LIBRARIES} )

    INSTALL( FILES ${includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/${PROJECT} )
    INSTALL(
    TARGETS ${TARGET_NAME}
    RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime     # .exe, .dll
    LIBRARY DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Runtime     # .so, mod.dll
    ARCHIVE DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Development # .a, .lib
    )

    IF( snark_BUILD_APPLICATIONS )
        ADD_SUBDIRECTORY( applications )
        EXECUTE_PROCESS( COMMAND rm -f ${SOURCE_CODE_BASE_DIR}/${PROJECT}/applications/test/disabled )
    ELSE( snark_BUILD_APPLICATIONS )
        EXECUTE_PROCESS( COMMAND touch ${SOURCE_CODE_BASE_DIR}/${PROJECT}/applications/test/disabled )
    ENDIF( snark_BUILD_APPLICATIONS )

    IF( snark_BUILD_TESTS )
        ADD_SUBDIRECTORY( test )
    ENDIF( snark_BUILD_TESTS )
ENDIF( NOT snark_build_control )
