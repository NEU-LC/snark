SOURCE_GROUP( hokuyo-to-csv FILES hokuyo-to-csv.cpp )
ADD_EXECUTABLE( hokuyo-to-csv hokuyo-to-csv.cpp )
TARGET_LINK_LIBRARIES( hokuyo-to-csv snark_hokuyo snark_hokuyo_detail ${snark_ALL_EXTERNAL_LIBRARIES} ${comma_ALL_EXTERNAL_LIBRARIES} )

INSTALL( TARGETS hokuyo-to-csv 
         RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR}
         COMPONENT Runtime )

