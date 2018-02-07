set( PROJECT "jai" )
set( TARGET_NAME snark_${PROJECT} )

if( WIN32 )
    message( FATAL "snark/sensors/cameras/jai: not implemented for windows" )
endif( WIN32 )

if( IS_64_BIT )
    set( JAI_GENICAM_LIB "lib64" )
    link_directories( "/usr/lib64" )
    link_directories( "/usr/local/lib64" )
else( IS_64_BIT )
    set( JAI_GENICAM_LIB "lib" )
endif( IS_64_BIT )

message( WARNING "attention! jai rpm installs /etc/profile.d/jaisdkenv64.sh, which sets CPATH=/usr/include/JAI:/usr/local/include, which is known to screws up compilation of other Gige cameras, e.g. basler due to enforcing incorrect includes for Genicam library; it currently is worked around, but beware" )

set( JAI_INCLUDE_DIR "/usr/include/JAI" CACHE PATH "jai camera include directory" )
set( JAI_GENICAM_ROOT "/usr/local/genicam" CACHE PATH "jai camera genicam directory" )
set( JAI_GENICAM_CACHE "/usr/local/xml/GenApi/Cache" CACHE PATH "jai camera genicam cache directory" )
set( JAI_GENICAM_VERSION_MAJOR "2" CACHE STRING "jai camera genicam version major" )
set( JAI_GENICAM_VERSION_MINOR "4" CACHE STRING "jai camera genicam version minor; todo: autodetect" )
set( JAI_GENICAM_LIB_DIR "${JAI_GENICAM_ROOT}/${JAI_GENICAM_LIB}" CACHE PATH "jai camera genicam lib directory" )
include_directories( ${JAI_INCLUDE_DIR} )
link_directories( ${JAI_GENICAM_LIB_DIR} )
set( JAI_SDK_LIBRARIES "JAIFactory" )

set( BINARY_DIR "${CMAKE_BINARY_DIR}/sensors/cameras/jai" )
add_custom_target( jai.conf ) # quick and dirty
add_custom_command( TARGET jai.conf POST_BUILD COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/jai-generate-ld-conf ARGS "${JAI_GENICAM_LIB_DIR}" )
install( FILES ${BINARY_DIR}/jai.conf DESTINATION /etc/ld.so.conf.d )
install( CODE "message( \"sensors/cameras/jai: running ldconfig\" )" )
install( CODE "execute_process( COMMAND ldconfig )" )
set( JAI_GENICAM_ROOT_SYMLINK_HACK "/opt/GenICam_v${JAI_GENICAM_VERSION_MAJOR}_${JAI_GENICAM_VERSION_MINOR}" )
install( CODE "message( \"sensors/cameras/jai: a hack: created symlink: ln -s ${JAI_GENICAM_ROOT} ${JAI_GENICAM_ROOT_SYMLINK_HACK}\" )" ) # HACK 
install( CODE "execute_process( COMMAND rm -rf ${JAI_GENICAM_ROOT_SYMLINK_HACK} )" ) # HACK
install( CODE "execute_process( COMMAND ln -s ${JAI_GENICAM_ROOT} ${JAI_GENICAM_ROOT_SYMLINK_HACK} )" ) # HACK

file( GLOB source *.cpp )
file( GLOB includes *.h )

source_group( ${TARGET_NAME} FILES ${source} ${includes} )

add_library( ${TARGET_NAME} ${source} ${includes} )
set_target_properties( ${TARGET_NAME} PROPERTIES ${snark_LIBRARY_PROPERTIES} )
target_link_libraries( ${TARGET_NAME} ${comma_ALL_LIBRARIES} ${comma_ALL_EXTERNAL_LIBRARIES} ${JAI_SDK_LIBRARIES} )
add_dependencies( ${TARGET_NAME} jai.conf )

install( FILES ${includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/sensors/cameras/${PROJECT} )
install(
   TARGETS ${TARGET_NAME}
   RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime     # .exe, .dll
   LIBRARY DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Runtime     # .so, mod.dll
   ARCHIVE DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Development # .a, .lib
)

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DJAI_GENICAM_ROOT=\"${JAI_GENICAM_ROOT}\"" )
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DJAI_GENICAM_ROOT_NAME=\"GENICAM_ROOT_V${JAI_GENICAM_VERSION_MAJOR}_${JAI_GENICAM_VERSION_MINOR}\"" )
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DJAI_GENICAM_CONFIG=\"${JAI_GENICAM_ROOT}/share/genicam/log/config/DefaultLogging.properties\"" )
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DJAI_GENICAM_CACHE=\"${JAI_GENICAM_CACHE}\"" )
set( CMAKE_CXX_FLAGS_RELWITHDEBINFO "-g3 -gdwarf-2 -g" )

add_subdirectory( applications )
