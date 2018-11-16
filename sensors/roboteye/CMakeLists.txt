set( PROJECT "roboteye" )
set( TARGET_NAME snark_${PROJECT} )

if( WIN32 )
    message( FATAL "not implemented" )
endif()

if( BUILD_SHARED_LIBS )
    message( FATAL_ERROR "\n${PROJECT} only builds against static libraries\ndisable BUILD_SHARED_LIBS or disable snark_build_sensors_roboteye" )
endif()

file( GLOB includes *.h)
file( GLOB sources *.cpp )

set( RobotEye_DIR "/opt/RobotEye/cmake" CACHE PATH "ocular roboteye cmake directory" )
find_package( RobotEye REQUIRED )

# The RobotEye library requires the Pleora ebus SDK
# PUREGEV_ROOT is set by RobotEyeConfig.cmake
set( OCULAR_ROBOTEYE_GENICAM_DIR ${PUREGEV_ROOT}/lib/genicam CACHE PATH "ocular roboteye genicam directory" )

message( STATUS "RobotEye_INCLUDE_DIR: " ${RobotEye_INCLUDE_DIR} )

include_directories( ${RobotEye_INCLUDE_DIR} )

source_group( ${TARGET_NAME} FILES ${sources} ${includes} )
add_library( ${TARGET_NAME} ${sources} ${includes} )
set_target_properties( ${TARGET_NAME} PROPERTIES ${snark_LIBRARY_PROPERTIES} )
target_link_libraries( ${TARGET_NAME} ${snark_ALL_LIBRARIES} ${RobotEye_LIBRARIES} )
install(
    TARGETS ${TARGET_NAME}
    RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime     # .exe, .dll
    LIBRARY DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Runtime     # .so, mod.dll
    ARCHIVE DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Development # .a, .lib
)

add_subdirectory( applications )
add_subdirectory( examples )
