set( PROJECT "vimba" )
set( TARGET_NAME snark_${PROJECT} )

file( RELATIVE_PATH project_path ${PROJECT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} )

if( WIN32 )
    message( FATAL_ERROR "snark/${project_path}: not implemented for windows" )
endif()

if( NOT IS_64_BIT )
    message( FATAL_ERROR "snark/${project_path}: only implemented for 64 bit systems" )
endif()

file( GLOB source *.cpp )
file( GLOB includes *.h )

source_group( ${TARGET_NAME} FILES ${source} ${includes} )

add_library( ${TARGET_NAME} ${source} ${includes} )
set_target_properties( ${TARGET_NAME} PROPERTIES ${snark_LIBRARY_PROPERTIES} )
target_link_libraries( ${TARGET_NAME} )

install( FILES ${includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/sensors/cameras/${PROJECT} )
install(
    TARGETS ${TARGET_NAME}
    LIBRARY DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Runtime     # .so, mod.dll
    ARCHIVE DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Development # .a, .lib
)

# We're using from c++11 features

execute_process( COMMAND ${CMAKE_CXX_COMPILER} -dumpversion
                 OUTPUT_VARIABLE GCC_VERSION
                 OUTPUT_STRIP_TRAILING_WHITESPACE )
if( GCC_VERSION VERSION_EQUAL 4.7 OR GCC_VERSION VERSION_GREATER 4.7 )
    message( STATUS "gcc ${GCC_VERSION} >= 4.7" )
    set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++11" )
else()
    message( STATUS "gcc ${GCC_VERSION} < 4.7" )
    set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++0x" )
endif()

add_subdirectory( applications )
