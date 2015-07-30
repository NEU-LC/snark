SET( PROJECT "Basler" )
SET( TARGET_NAME Ark${PROJECT} )

if( WIN32 )
    message( FATAL "not implemented" )
endif( WIN32 )

SET( BASLER_PYLON_DIR /opt/pylon CACHE PATH "basler pylon library installation directory" )
SET( BASLER_PYLON_GENICAM_DIR ${BASLER_PYLON_DIR}/genicam CACHE PATH "basler genicam directory" )
SET( BASLER_PYLON_MAJOR_VERSION 2 CACHE STRING "major version of basler pylon libraries" )
SET( BASLER_PYLON_MINOR_VERSION 3 CACHE STRING "minor version of basler pylon libraries" )

# voodoo, see pylon sample makefiles
INCLUDE_DIRECTORIES( BEFORE ${BASLER_PYLON_GENICAM_DIR}/library/CPP/include )
INCLUDE_DIRECTORIES( BEFORE ${BASLER_PYLON_DIR}/include )
LINK_DIRECTORIES( ${BASLER_PYLON_DIR}/lib64 )
LINK_DIRECTORIES( ${BASLER_PYLON_DIR}/lib )
LINK_DIRECTORIES( ${BASLER_PYLON_GENICAM_DIR}/bin/Linux64_x64 )
LINK_DIRECTORIES( ${BASLER_PYLON_GENICAM_DIR}/bin/Linux64_x64/GenApi/Generic )
LINK_DIRECTORIES( ${BASLER_PYLON_GENICAM_DIR}/bin/Linux32_i86 )
LINK_DIRECTORIES( ${BASLER_PYLON_GENICAM_DIR}/bin/Linux32_i86/GenApi/Generic )
#SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_GIGE" )
#SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,-E" )
SET( CMAKE_CXX_FLAGS "-DUSE_GIGE -Wl,-E" )

file( GLOB source *.cpp )
file( GLOB includes *.h )
source_group( ${TARGET_NAME} FILES ${source} ${includes} )
add_subdirectory( applications )
