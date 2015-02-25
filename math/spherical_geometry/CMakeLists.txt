SET( PROJECT "spherical_geometry" )

SET( TARGET_NAME snark_math_${PROJECT} )

FILE( GLOB includes *.h)

FILE( GLOB sources *.cpp )

source_group( ${TARGET_NAME} FILES ${sources} ${includes} )

#add_library( ${TARGET_NAME} OBJECT ${sources} ${includes} )    //OBJECT doesn't work on cmake 2.8.2
add_library( ${TARGET_NAME} ${sources} ${includes} )

install( FILES ${includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/math/${PROJECT} )
