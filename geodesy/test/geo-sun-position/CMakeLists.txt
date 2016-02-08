MACRO( enable_testing )
    EXECUTE_PROCESS( COMMAND rm -f ${CMAKE_CURRENT_SOURCE_DIR}/disabled )
    MESSAGE( STATUS "${CMAKE_CURRENT_SOURCE_DIR}/disabled has been removed: testing is enabled" )
ENDMACRO( enable_testing )

MACRO( disable_testing )
    EXECUTE_PROCESS( COMMAND touch ${CMAKE_CURRENT_SOURCE_DIR}/disabled )
    MESSAGE( STATUS "disabled file has been dropped in ${CMAKE_CURRENT_SOURCE_DIR}: testing is disabled" )
ENDMACRO( disable_testing )

MACRO( check_python_module module )
    EXECUTE_PROCESS( COMMAND ${PYTHON} -c "import ${module};"
        RESULT_VARIABLE MODULE_EXIT_CODE
        ERROR_VARIABLE MODULE_ERROR
        OUTPUT_STRIP_TRAILING_WHITESPACE )
    IF( MODULE_EXIT_CODE MATCHES 0 )
        MESSAGE( STATUS "found python ${module} module" )
    ELSE( MODULE_EXIT_CODE MATCHES 0 )
        MESSAGE( WARNING "python ${module} module not found" )
        disable_testing()
        RETURN()
    ENDIF( MODULE_EXIT_CODE MATCHES 0 )
ENDMACRO( check_python_module )

IF( NOT snark_build_python )
    disable_testing()
ENDIF()

FIND_PROGRAM( PYTHON "python" )

IF( NOT PYTHON )
    MESSAGE( WARNING "python not found" )
    disable_testing()
    RETURN()
ENDIF( NOT PYTHON )

check_python_module( comma )
check_python_module( numpy )
check_python_module( Pysolar )

enable_testing()
