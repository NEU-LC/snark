SET( dir ${SOURCE_CODE_BASE_DIR}/math/applications )
FILE( GLOB source   ${dir}/*.cpp)
FILE( GLOB includes ${dir}/*.h)
SOURCE_GROUP( ${TARGET_NAME} FILES ${source} ${includes} )

ADD_EXECUTABLE( math-deg2rad math-deg2rad.cpp )
TARGET_LINK_LIBRARIES( math-deg2rad ${comma_ALL_LIBRARIES} )

ADD_EXECUTABLE( math-rad2deg math-rad2deg.cpp )
TARGET_LINK_LIBRARIES( math-rad2deg ${comma_ALL_LIBRARIES} )

ADD_EXECUTABLE( math-rotation-convert math-rotation-convert.cpp )
TARGET_LINK_LIBRARIES( math-rotation-convert snark_math ${comma_ALL_LIBRARIES} )

ADD_EXECUTABLE( points-frame frame.cpp points-frame.cpp )
TARGET_LINK_LIBRARIES( points-frame ${comma_ALL_LIBRARIES} ${snark_ALL_LIBRARIES} )

INSTALL( TARGETS math-deg2rad math-rad2deg math-rotation-convert points-frame
         RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/${snark_INSTALL_BIN_DIR}
         COMPONENT Runtime )
