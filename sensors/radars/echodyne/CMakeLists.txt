set( PROJECT "echodyne" )
set( TARGET_NAME snark_${PROJECT} )

if( WIN32 )
    message( FATAL_ERROR "not implemented (but could be)" )
endif()

string( REGEX MATCH "17" version_match ${CXX_STANDARD_TO_USE} )
if( "${version_match}" STREQUAL "" )
    message( FATAL_ERROR "\nradars/echodyne requires C++17\n"
        "Set CXX_STANDARD_TO_USE (or disable this module)"
        )
endif()

set( BNET_DIR "/opt/echodyne" CACHE PATH "Path to Echodyne BNET API" )
find_package( BNET REQUIRED )

file( GLOB source *.cpp )
file( GLOB includes *.h )

source_group( ${TARGET_NAME} FILES ${source} ${includes} )

add_library( ${TARGET_NAME} ${source} ${includes} )
set_target_properties( ${TARGET_NAME} PROPERTIES ${snark_LIBRARY_PROPERTIES} )
target_link_libraries( ${TARGET_NAME} snark_math ${snark_ALL_EXTERNAL_LIBRARIES} )

include_directories( ${BNET_API_INCLUDE_DIR} ${BNET_API_DIR}/bnet_interface/inc ${BNET_API_DIR}/bnet_reader/inc )

install( FILES ${includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/sensors/radars/${PROJECT} )
install(
   TARGETS ${TARGET_NAME}
   RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime     # .exe, .dll
   LIBRARY DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Runtime     # .so, mod.dll
   ARCHIVE DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Development # .a, .lib
)
                 
add_subdirectory( applications )
