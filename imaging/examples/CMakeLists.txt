SET( dir ${SOURCE_CODE_BASE_DIR}/imaging/examples )

ADD_EXECUTABLE( region-properties-demo region-properties-demo.cpp region_properties.h region_properties.cpp )
TARGET_LINK_LIBRARIES( region-properties-demo snark_imaging ${comma_ALL_LIBRARIES} ${comma_ALL_LIBRARIES} ${OpenCV_LIBS} )

ADD_EXECUTABLE( frequency-domain-demo frequency-domain-demo.cpp frequency_domain.h frequency_domain.cpp )
TARGET_LINK_LIBRARIES( frequency-domain-demo snark_imaging ${comma_ALL_LIBRARIES} ${comma_ALL_LIBRARIES} ${OpenCV_LIBS} )

# does not build anymore...
#if( snark_build_imaging_stereo ) # todo: add proper cmake to stereo
#    ADD_EXECUTABLE( stereo-demo stereo-demo.cpp )
#    TARGET_LINK_LIBRARIES( stereo-demo snark_imaging snark_math ${comma_ALL_LIBRARIES} ${comma_ALL_LIBRARIES} ${OpenCV_LIBS} )
#endif( snark_build_imaging_stereo )
