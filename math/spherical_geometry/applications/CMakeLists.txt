ADD_EXECUTABLE( sphere-sample sphere-sample.cpp )
TARGET_LINK_LIBRARIES( sphere-sample ${comma_ALL_LIBRARIES} snark_math_spherical_geometry ${snark_ALL_EXTERNAL_LIBRARIES} )
INSTALL( TARGETS sphere-sample RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )
