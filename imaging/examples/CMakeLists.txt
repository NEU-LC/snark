SET( dir ${SOURCE_CODE_BASE_DIR}/imaging/examples )
FILE( GLOB source   ${dir}/*.cpp)
FILE( GLOB includes ${dir}/*.h)
SOURCE_GROUP( ${TARGET_NAME} FILES ${source} ${includes} )

ADD_EXECUTABLE( regionprops-demo regionprops-demo.cpp )
TARGET_LINK_LIBRARIES( regionprops-demo snark_imaging ${comma_ALL_LIBRARIES} ${comma_ALL_LIBRARIES} ${OpenCV_LIBS} )

ADD_EXECUTABLE( frequency-demo frequency-demo.cpp )
TARGET_LINK_LIBRARIES( frequency-demo snark_imaging ${comma_ALL_LIBRARIES} ${comma_ALL_LIBRARIES} ${OpenCV_LIBS} )

ADD_EXECUTABLE( stereo-demo stereo-demo.cpp )
TARGET_LINK_LIBRARIES( stereo-demo snark_imaging snark_math ${comma_ALL_LIBRARIES} ${comma_ALL_LIBRARIES} ${OpenCV_LIBS} )
