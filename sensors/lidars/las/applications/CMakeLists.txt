SOURCE_GROUP( las-to-csv FILES las-to-csv.cpp )
ADD_EXECUTABLE( las-to-csv las-to-csv.cpp )
TARGET_LINK_LIBRARIES( las-to-csv ${snark_ALL_EXTERNAL_LIBRARIES} ${comma_ALL_EXTERNAL_LIBRARIES} )
set_target_properties( las-to-csv PROPERTIES LINK_FLAGS_RELEASE -s )
INSTALL( TARGETS las-to-csv RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

