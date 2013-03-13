SOURCE_GROUP( sick-ldmrs-stream FILES sick-ldmrs-stream.cpp )
ADD_EXECUTABLE( sick-ldmrs-stream sick-ldmrs-stream.cpp )
TARGET_LINK_LIBRARIES( sick-ldmrs-stream snark_sick snark_timing ${snark_ALL_EXTERNAL_LIBRARIES} )

SOURCE_GROUP( sick-ldmrs-to-csv FILES sick-ldmrs-to-csv.cpp )
ADD_EXECUTABLE( sick-ldmrs-to-csv sick-ldmrs-to-csv.cpp )
TARGET_LINK_LIBRARIES( sick-ldmrs-to-csv snark_sick snark_timing snark_math ${snark_ALL_EXTERNAL_LIBRARIES} )

INSTALL( TARGETS sick-ldmrs-stream sick-ldmrs-to-csv
         RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR}
         COMPONENT Runtime )
