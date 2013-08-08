SET( dir ${SOURCE_CODE_BASE_DIR}/imaging/applications )
FILE( GLOB source   ${dir}/*.cpp)
FILE( GLOB includes ${dir}/*.h)

FILE( GLOB stereo_source ${dir}/stereo/*.cpp )
FILE( GLOB stereo_includes ${dir}/stereo/*.h )

SOURCE_GROUP( ${TARGET_NAME} FILES ${source} ${includes} )

ADD_EXECUTABLE( image-undistort-map image-undistort-map.cpp )
TARGET_LINK_LIBRARIES( image-undistort-map ${comma_ALL_LIBRARIES} ${OpenCV_LIBS} )

ADD_EXECUTABLE( cv-cat cv-cat.cpp )
TARGET_LINK_LIBRARIES( cv-cat snark_imaging ${comma_ALL_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} tbb )

ADD_EXECUTABLE( image-accumulate image-accumulate.cpp )
TARGET_LINK_LIBRARIES( image-accumulate snark_imaging ${comma_ALL_LIBRARIES} ${OpenCV_LIBS} tbb )

ADD_EXECUTABLE( stereo-to-points stereo-to-points.cpp ${stereo_source}  )
TARGET_LINK_LIBRARIES( stereo-to-points snark_imaging snark_math ${comma_ALL_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} )

INSTALL( TARGETS image-undistort-map cv-cat image-accumulate stereo-to-points 
         RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR}
         COMPONENT Runtime )
