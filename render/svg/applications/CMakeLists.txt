ADD_EXECUTABLE( csv-to-svg csv-to-svg.cpp )
TARGET_LINK_LIBRARIES( csv-to-svg snark_svg ${comma_ALL_LIBRARIES} )
INSTALL( TARGETS csv-to-svg RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )
