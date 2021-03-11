set( snark_geo_sun_position_test_disabled_file "${CMAKE_CURRENT_SOURCE_DIR}/disabled" )
macro( enable_testing )
    if( EXISTS "${snark_geo_sun_position_test_disabled_file}" )
        execute_process( COMMAND rm -f "${snark_geo_sun_position_test_disabled_file}" )
        message( STATUS "${snark_geo_sun_position_test_disabled_file} has been removed: testing is enabled" )
    endif()
endmacro()

macro( disable_testing )
    if( NOT EXISTS "${snark_geo_sun_position_test_disabled_file}" )
        execute_process( COMMAND touch "${snark_geo_sun_position_test_disabled_file}" )
        message( STATUS "${snark_geo_sun_position_test_disabled_file} added: testing is disabled" )
    endif()
endmacro()

MACRO( check_python_module module )
    EXECUTE_PROCESS( COMMAND ${PYTHON} -c "import ${module};"
        RESULT_VARIABLE MODULE_EXIT_CODE
        ERROR_VARIABLE MODULE_ERROR
        OUTPUT_STRIP_TRAILING_WHITESPACE )
    IF( MODULE_EXIT_CODE MATCHES 0 )
        MESSAGE( STATUS "found python ${module} module" )
    ELSE( MODULE_EXIT_CODE MATCHES 0 )
        MESSAGE( STATUS "python ${module} module not found" )
        disable_testing()
        RETURN()
    ENDIF( MODULE_EXIT_CODE MATCHES 0 )
ENDMACRO( check_python_module )

IF( NOT snark_build_python )
    disable_testing()
    return()
ENDIF()

FIND_PROGRAM( PYTHON "python" )

IF( NOT PYTHON )
    MESSAGE( STATUS "python not found" )
    disable_testing()
    RETURN()
ENDIF( NOT PYTHON )

check_python_module( comma )
check_python_module( numpy )
check_python_module( Pysolar )

enable_testing()
