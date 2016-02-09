MACRO( enable_python_testing )
    EXECUTE_PROCESS( COMMAND rm -f ${CMAKE_CURRENT_SOURCE_DIR}/disabled )
    MESSAGE( STATUS "${CMAKE_CURRENT_SOURCE_DIR}/disabled has been removed: testing python modules and applications is enabled" )
ENDMACRO( enable_python_testing )

MACRO( disable_python_testing )
    EXECUTE_PROCESS( COMMAND touch ${CMAKE_CURRENT_SOURCE_DIR}/disabled )
    MESSAGE( STATUS "disabled file has been dropped in ${CMAKE_CURRENT_SOURCE_DIR}: testing python modules and applications is disabled" )
ENDMACRO( disable_python_testing )

MACRO( check_python_module module )
    EXECUTE_PROCESS( COMMAND ${PYTHON} -c "import ${module}; print( ${module}.__version__ );"
        RESULT_VARIABLE MODULE_EXIT_CODE
        OUTPUT_VARIABLE MODULE_VERSION_STRING
        ERROR_VARIABLE MODULE_ERROR
        OUTPUT_STRIP_TRAILING_WHITESPACE )
    IF( MODULE_EXIT_CODE MATCHES 0 )
        MESSAGE( STATUS "found python ${module} module version ${MODULE_VERSION_STRING}" )
    ELSE( MODULE_EXIT_CODE MATCHES 0 )
        MESSAGE( WARNING "python ${module} module not found; install it or disable BUILD_PYTHON_PACKAGES" )
        MESSAGE( STATUS "snark/python modules and scripts will not be installed" )
        disable_python_testing()
        RETURN()
    ENDIF( MODULE_EXIT_CODE MATCHES 0 )
ENDMACRO( check_python_module )


IF( NOT snark_build_python )
    disable_python_testing()
    RETURN()
ENDIF( NOT snark_build_python )

FIND_PROGRAM( PYTHON "python" )

IF( NOT PYTHON )
    MESSAGE( WARNING "python not found" )
    MESSAGE( STATUS "snark/python modules and scripts will not be installed" )
    disable_python_testing()
    RETURN()
ENDIF( NOT PYTHON )

check_python_module( comma )
check_python_module( numpy )
check_python_module( cv2 )
enable_python_testing()

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

IF( ADD_PYTHON_PACKAGES_TO_RPM )
    SET( PYTHON_TMP "/tmp/snark/python_modules_for_rpm_install" )
    INSTALL( DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/ DESTINATION ${PYTHON_TMP}
        FILES_MATCHING PATTERN "*.py"
        PATTERN test EXCLUDE
        PATTERN output EXCLUDE
        PATTERN stats EXCLUDE
        PATTERN build EXCLUDE
        PATTERN examples EXCLUDE )
ENDIF( ADD_PYTHON_PACKAGES_TO_RPM )

ADD_SUBDIRECTORY( snark )
