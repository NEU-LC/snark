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

file( GLOB source *.cpp )
file( GLOB includes *.h )

source_group( ${TARGET_NAME} FILES ${source} ${includes} )

add_library( ${TARGET_NAME} ${source} ${includes} )
set_target_properties( ${TARGET_NAME} PROPERTIES ${snark_LIBRARY_PROPERTIES} )
target_link_libraries( ${TARGET_NAME} snark_math ${snark_ALL_EXTERNAL_LIBRARIES} )

set( Echodyne_API "/opt/echodyne/API" CACHE PATH "Path to Echodyne API" )
if( NOT EXISTS ${Echodyne_API} )
    message( FATAL_ERROR "\nPlease install the Echodyne API at ${Echodyne_API} "
        "or set Echodyne_API to point to the API installation directory.\n"
        "The API is available from https://portal.echodyne.com/\n"
        "The code in this project is currently developed and tested against Rev 10 of the API."
        )
endif()

include_directories( ${Echodyne_API}/include ${Echodyne_API}/bnet_interface/inc ${Echodyne_API}/bnet_reader/inc )
link_directories( ${Echodyne_API}/bnet_interface/lib ${Echodyne_API}/bnet_reader/lib )

install( FILES ${includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/sensors/radars/${PROJECT} )
install(
   TARGETS ${TARGET_NAME}
   RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime     # .exe, .dll
   LIBRARY DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Runtime     # .so, mod.dll
   ARCHIVE DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Development # .a, .lib
)
                 
add_subdirectory( applications )
