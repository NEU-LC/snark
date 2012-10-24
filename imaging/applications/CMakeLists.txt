SET( dir ${SOURCE_CODE_BASE_DIR}/imaging/applications )
FILE( GLOB source   ${dir}/*.cpp)
FILE( GLOB includes ${dir}/*.h)
SOURCE_GROUP( ${TARGET_NAME} FILES ${source} ${includes} )

ADD_EXECUTABLE( image-undistort-map image-undistort-map.cpp )
TARGET_LINK_LIBRARIES( image-undistort-map ${comma_ALL_LIBRARIES} ${comma_ALL_LIBRARIES} ${OpenCV_LIBS} )

ADD_EXECUTABLE( cv-cat cv-cat.cpp )
TARGET_LINK_LIBRARIES( cv-cat snark_imaging ${comma_ALL_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} tbb )

ADD_EXECUTABLE( image-accumulate image-accumulate.cpp )
TARGET_LINK_LIBRARIES( image-accumulate snark_imaging ${comma_ALL_LIBRARIES} ${comma_ALL_LIBRARIES} ${OpenCV_LIBS} tbb )

ADD_EXECUTABLE( regionprops-demo regionprops-demo.cpp )
TARGET_LINK_LIBRARIES( regionprops-demo snark_imaging ${comma_ALL_LIBRARIES} ${comma_ALL_LIBRARIES} ${OpenCV_LIBS} )

ADD_EXECUTABLE( frequency-demo frequency-demo.cpp )
TARGET_LINK_LIBRARIES( frequency-demo snark_imaging ${comma_ALL_LIBRARIES} ${comma_ALL_LIBRARIES} ${OpenCV_LIBS} )

INSTALL( TARGETS image-undistort-map cv-cat image-accumulate
         RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/${snark_INSTALL_BIN_DIR}
         COMPONENT Runtime )
