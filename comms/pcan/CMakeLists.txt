if( WIN32 )
    message( FATAL_ERROR "pcan: not implemented on windows" )
endif( WIN32 )

set( PROJECT "pcan" )
set( DIR ${SOURCE_CODE_BASE_DIR}/comms/${PROJECT} )
file( GLOB includes ${DIR}/*.h )
add_custom_target( snark_${PROJECT} ${includes} )
# todo target_link_libraries( snark_${PROJECT} pcan )
install( FILES ${includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/comms/${PROJECT} )

if( snark_BUILD_APPLICATIONS )
    add_subdirectory( applications )
endif( snark_BUILD_APPLICATIONS )
