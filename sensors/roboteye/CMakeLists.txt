set( PROJECT "roboteye" )
set( TARGET_NAME snark_${PROJECT} )

if( BUILD_SHARED_LIBS )
    message( FATAL_ERROR "\n${PROJECT} only builds against static libraries\ndisable BUILD_SHARED_LIBS or disable snark_build_sensors_roboteye" )
endif()

file( GLOB includes *.h)
file( GLOB sources *.cpp )

## from ocular example
# Here you can point to a specific RobotEye library location if needed
# This directory must contain the RobotEyeConfig.cmake file
# Just set RobotEye_DIR to the desired location before calling #FIND_PACKAGE
# IF(WIN32)
#   if(${CMAKE_CL_64})
# #     MESSAGE(STATUS "Architecture: x64")
#     set(RobotEye_DIR "C:/Program Files/RobotEye/cmake")
#   else()
# #     MESSAGE(STATUS "Architecture: x86")
#     set(RobotEye_DIR "C:/Program Files (x86)/RobotEye/cmake")
#   endif(${CMAKE_CL_64})
# else()
#     set(RobotEye_DIR "/opt/RobotEye/cmake")
# endif(WIN32)

set( RobotEye_DIR "/opt/RobotEye/cmake" CACHE PATH "Ocular RobotEye cmake directory" )
FIND_PACKAGE( RobotEye REQUIRED )

MESSAGE( STATUS "RobotEye_LIBRARIES: " ${RobotEye_LIBRARIES} )
MESSAGE( STATUS "RobotEye_INCLUDE_DIR: " ${RobotEye_INCLUDE_DIR} )

INCLUDE_DIRECTORIES( ${RobotEye_INCLUDE_DIR} )
##

source_group( ${TARGET_NAME} FILES ${sources} ${includes} )
add_library( ${TARGET_NAME} ${sources} ${includes} )
set_target_properties( ${TARGET_NAME} PROPERTIES ${snark_LIBRARY_PROPERTIES} )
target_link_libraries( ${TARGET_NAME} ${snark_ALL_EXTERNAL_LIBRARIES} ${RobotEye_LIBRARIES} )
install(
    TARGETS ${TARGET_NAME}
    RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime     # .exe, .dll
    LIBRARY DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Runtime     # .so, mod.dll
    ARCHIVE DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Development # .a, .lib
)

add_subdirectory( applications )
add_subdirectory( examples )
