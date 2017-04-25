SET( dir ${SOURCE_CODE_BASE_DIR}/imaging/applications )
FILE( GLOB source   ${dir}/*.cpp)
FILE( GLOB includes ${dir}/*.h)

FILE( GLOB stereo_source ${dir}/stereo/*.cpp )
FILE( GLOB stereo_includes ${dir}/stereo/*.h )

SOURCE_GROUP( ${TARGET_NAME} FILES ${source} ${includes} )

ADD_EXECUTABLE( image-undistort-map image-undistort-map.cpp )
TARGET_LINK_LIBRARIES( image-undistort-map snark_imaging ${comma_ALL_LIBRARIES} ${OpenCV_LIBS} )

ADD_EXECUTABLE( cv-cat cv-cat.cpp )
TARGET_LINK_LIBRARIES( cv-cat snark_imaging ${comma_ALL_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} tbb )

ADD_EXECUTABLE( cv-calc cv-calc.cpp )
TARGET_LINK_LIBRARIES( cv-calc snark_imaging ${comma_ALL_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} )

add_executable( image-from-csv image-from-csv.cpp )
target_link_libraries( image-from-csv snark_imaging ${comma_ALL_LIBRARIES} ${OpenCV_LIBS} )
install( TARGETS image-from-csv RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

ADD_EXECUTABLE( image-to-csv image-to-csv.cpp )
TARGET_LINK_LIBRARIES( image-to-csv snark_imaging ${comma_ALL_LIBRARIES} ${OpenCV_LIBS} )

ADD_EXECUTABLE( image-accumulate image-accumulate.cpp )
TARGET_LINK_LIBRARIES( image-accumulate snark_imaging ${comma_ALL_LIBRARIES} ${OpenCV_LIBS} tbb )

ADD_EXECUTABLE( stereo-to-points stereo-to-points.cpp ${stereo_source}  )
TARGET_LINK_LIBRARIES( stereo-to-points snark_imaging snark_math ${comma_ALL_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} )

INSTALL( TARGETS image-undistort-map cv-cat cv-calc image-accumulate stereo-to-points image-to-csv
         RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR}
         COMPONENT Runtime )
         
add_executable( image-pinhole image-pinhole.cpp )
target_link_libraries( image-pinhole snark_imaging ${comma_ALL_LIBRARIES} ${OpenCV_LIBS} ) #target_link_libraries( image-pinhole snark_imaging ${comma_ALL_LIBRARIES} ${OpenCV_LIBS} tbb )
install( TARGETS image-pinhole RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

add_executable( image-pinhole-calibrate image-pinhole-calibrate.cpp )
target_link_libraries( image-pinhole-calibrate snark_imaging ${comma_ALL_LIBRARIES} ${OpenCV_LIBS} )
install( TARGETS image-pinhole-calibrate RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )


