SOURCE_GROUP( asd-control FILES asd-control.cpp )
ADD_EXECUTABLE( asd-control asd-control.cpp )
set_target_properties( asd-control PROPERTIES LINK_FLAGS_RELEASE -s )
TARGET_LINK_LIBRARIES( asd-control snark_asd ${snark_ALL_EXTERNAL_LIBRARIES} )
INSTALL( TARGETS asd-control RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

SOURCE_GROUP( asd-to-csv FILES asd-to-csv.cpp )
ADD_EXECUTABLE( asd-to-csv asd-to-csv.cpp )
set_target_properties( asd-to-csv PROPERTIES LINK_FLAGS_RELEASE -s )
TARGET_LINK_LIBRARIES( asd-to-csv snark_asd ${snark_ALL_EXTERNAL_LIBRARIES} )
INSTALL( TARGETS asd-to-csv RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

