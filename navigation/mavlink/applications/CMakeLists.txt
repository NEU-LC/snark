ADD_EXECUTABLE( mavlink-to-csv mavlink-to-csv.cpp )
TARGET_LINK_LIBRARIES( mavlink-to-csv )

INSTALL( TARGETS mavlink-to-csv
         RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR}
         COMPONENT Runtime )
