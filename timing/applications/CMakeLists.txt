SOURCE_GROUP( csv-time-adjust FILES csv-time-adjust.cpp )
ADD_EXECUTABLE( csv-time-adjust csv-time-adjust.cpp )
TARGET_LINK_LIBRARIES( csv-time-adjust snark_timing ${snark_ALL_EXTERNAL_LIBRARIES} )

INSTALL( TARGETS csv-time-adjust RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )
