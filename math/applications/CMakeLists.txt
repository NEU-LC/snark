SET( dir ${SOURCE_CODE_BASE_DIR}/math/applications )
FILE( GLOB source   ${dir}/*.cpp)
FILE( GLOB includes ${dir}/*.h)
SOURCE_GROUP( ${TARGET_NAME} FILES ${source} ${includes} )

ADD_EXECUTABLE( math-deg2rad math-deg2rad.cpp )
TARGET_LINK_LIBRARIES( math-deg2rad ${comma_ALL_LIBRARIES} boost_atomic )
INSTALL( TARGETS math-deg2rad RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

ADD_EXECUTABLE( math-rad2deg math-rad2deg.cpp )
TARGET_LINK_LIBRARIES( math-rad2deg ${comma_ALL_LIBRARIES} boost_atomic )
INSTALL( TARGETS math-rad2deg RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

ADD_EXECUTABLE( math-rotation-convert math-rotation-convert.cpp )
TARGET_LINK_LIBRARIES( math-rotation-convert snark_math ${comma_ALL_LIBRARIES} boost_atomic )
INSTALL( TARGETS math-rotation-convert RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

IF( CMAKE_SIZEOF_VOID_P EQUAL 4 )
    # disable eigen vectorization on 32 bit machines
    # see http://eigen.tuxfamily.org/index.php?title=FAQ#Vectorization
    ADD_DEFINITIONS( -DEIGEN_DONT_VECTORIZE )
    ADD_DEFINITIONS( -DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT )
    ADD_DEFINITIONS( -DEIGEN_DONT_ALIGN_STATICALLY )
ENDIF()

ADD_EXECUTABLE( points-frame frame.cpp points-frame.cpp )
TARGET_LINK_LIBRARIES( points-frame snark_math ${comma_ALL_LIBRARIES} )
INSTALL( TARGETS points-frame RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

ADD_EXECUTABLE( points-to-cartesian points-to-cartesian.cpp )
TARGET_LINK_LIBRARIES( points-to-cartesian snark_math ${comma_ALL_LIBRARIES} boost_atomic )
INSTALL( TARGETS points-to-cartesian RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

ADD_EXECUTABLE( points-to-polar points-to-polar.cpp )
TARGET_LINK_LIBRARIES( points-to-polar snark_math ${comma_ALL_LIBRARIES} boost_atomic )
INSTALL( TARGETS points-to-polar RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

ADD_EXECUTABLE( points-calc points-calc.cpp )
TARGET_LINK_LIBRARIES( points-calc snark_math ${comma_ALL_LIBRARIES} boost_atomic )
INSTALL( TARGETS points-calc RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )
