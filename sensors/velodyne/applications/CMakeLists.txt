SOURCE_GROUP( velodyne-stream-example FILES velodyne-stream-example.cpp )
ADD_EXECUTABLE( velodyne-stream-example velodyne-stream-example.cpp )
TARGET_LINK_LIBRARIES( velodyne-stream-example snark_velodyne ${snark_ALL_EXTERNAL_LIBRARIES} )

SOURCE_GROUP( velodyne-to-csv FILES velodyne-to-csv.cpp )
ADD_EXECUTABLE( velodyne-to-csv velodyne-to-csv.cpp )

TARGET_LINK_LIBRARIES( velodyne-to-csv snark_velodyne ${snark_ALL_EXTERNAL_LIBRARIES} )

INSTALL( TARGETS velodyne-to-csv 
         RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/${snark_INSTALL_BIN_DIR}
         COMPONENT Runtime )
