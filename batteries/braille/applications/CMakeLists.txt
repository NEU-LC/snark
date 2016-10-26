ADD_EXECUTABLE( braille-to-csv braille-to-csv.cpp )
TARGET_LINK_LIBRARIES( braille-to-csv ${snark_ALL_EXTERNAL_LIBRARIES} )

INSTALL( TARGETS braille-to-csv
         RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/${snark_INSTALL_BIN_DIR}
         COMPONENT Runtime )

