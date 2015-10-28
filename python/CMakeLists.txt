IF( snark_build_python )
    EXECUTE_PROCESS( COMMAND rm -f ${SOURCE_CODE_BASE_DIR}/python/disabled )
ELSE( snark_build_python )
    EXECUTE_PROCESS( COMMAND touch ${SOURCE_CODE_BASE_DIR}/python/disabled )
ENDIF( snark_build_python )

IF( snark_build_python )
    FIND_PROGRAM( PYTHON "python" )

    IF( NOT PYTHON )
        message( FATAL_ERROR "python modules and scripts cannot be installed: python not found" )
    ENDIF( NOT PYTHON )

    SET( PYTHON_PACKAGE_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX} CACHE STRING "install prefix for python packages, leave empty for python default prefix" )

    SET( INSTALL_PREFIX_OPTION "" )
    IF( PYTHON_PACKAGE_INSTALL_PREFIX )
        SET( INSTALL_PREFIX_OPTION "--prefix=${PYTHON_PACKAGE_INSTALL_PREFIX}" )
        IF( NOT WIN32 )
            SET( INSTALL_PREFIX_OPTION "${INSTALL_PREFIX_OPTION} --exec-prefix=${PYTHON_PACKAGE_INSTALL_PREFIX}" )
        ENDIF( NOT WIN32 )
    ENDIF( PYTHON_PACKAGE_INSTALL_PREFIX )

    SET( SETUP_PY "${CMAKE_CURRENT_SOURCE_DIR}/setup.py" )

    SET( BUILD_BASE "${CMAKE_CURRENT_BINARY_DIR}/build" )

    ADD_CUSTOM_TARGET( python_build ALL
                        ${PYTHON} ${SETUP_PY} build --build-base ${BUILD_BASE}
                        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} )

    INSTALL( CODE "EXECUTE_PROCESS(
                        COMMAND ${PYTHON} ${SETUP_PY} build --build-base ${BUILD_BASE} install ${INSTALL_PREFIX_OPTION}
                        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} )" )

    SET_DIRECTORY_PROPERTIES( PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES build )
ENDIF( snark_build_python )
