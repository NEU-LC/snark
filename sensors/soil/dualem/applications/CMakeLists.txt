SOURCE_GROUP( dualem-to-csv FILES dualem-to-csv.cpp )
ADD_EXECUTABLE( dualem-to-csv dualem-to-csv.cpp )
TARGET_LINK_LIBRARIES( dualem-to-csv snark_dualem ${snark_ALL_EXTERNAL_LIBRARIES} )
INSTALL( TARGETS dualem-to-csv 
         RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR}
         COMPONENT Runtime )
INSTALL( PROGRAMS dualem-publishd DESTINATION ${snark_INSTALL_BIN_DIR} )
