set( PROJECT "ipx" )
set( TARGET_NAME snark_${PROJECT} )
if( WIN32 )
    message( FATAL_ERROR "not implemented" )
endif( WIN32 )
set( IPX_SDK_DIR "" CACHE PATH "ipx sdk directory" )
find_package( IpxCameraSDK REQUIRED PATHS "${IPX_SDK_DIR}" )
add_custom_target( ipx.conf ) # quick and dirty
add_custom_command( TARGET ipx.conf
                    POST_BUILD
                    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/ipx-generate-ld-conf ARGS "${IpxCameraSDK_LIBRARY_DIR}" )
install( FILES ${CMAKE_BINARY_DIR}/sensors/cameras/ipx/ipx.conf DESTINATION /etc/ld.so.conf.d )
install( CODE "message( \"sensors/cameras/ipx: running ldconfig\" )" )
install( CODE "execute_process( COMMAND ldconfig )" )
include_directories( BEFORE SYSTEM ${IpxCameraSDK_INCLUDE_DIR} )
link_directories( "${IpxCameraSDK_LIBRARY_DIR}" )
link_directories( "${IpxCameraSDK_GenLIB_DIR}" )
file( GLOB source *.cpp )
file( GLOB includes *.h )
source_group( ${TARGET_NAME} FILES ${source} ${includes} )
add_library( ${TARGET_NAME} ${source} ${includes} )
set_target_properties( ${TARGET_NAME} PROPERTIES ${snark_LIBRARY_PROPERTIES} )
set_target_properties( ${TARGET_NAME} PROPERTIES LINK_FLAGS_RELEASE -s )
target_link_libraries( ${TARGET_NAME}
                       snark_imaging
                       ${comma_ALL_LIBRARIES}
                       ${comma_ALL_EXTERNAL_LIBRARIES}
                       ${snark_ALL_EXTERNAL_LIBRARIES} )
if( ${IpxCameraSDK_VERSION} STRLESS "1.5.0.54" ) # quick and dirty
    target_link_libraries( ${TARGET_NAME} ippLib17 )
endif( ${IpxCameraSDK_VERSION} STRLESS "1.5.0.54" )
target_link_libraries( ${TARGET_NAME}
                       IpxCameraApi
                       IpxCameraGuiApi
                       IpxDemosaicing
                       IpxDisplayQt
                       IpxImageApi
                       IpxImageConverter
                       IpxImageSerializer
                       IpxImageUnpacker
                       IpxMisc )
add_dependencies( ${TARGET_NAME} ipx.conf )
add_subdirectory( applications )
