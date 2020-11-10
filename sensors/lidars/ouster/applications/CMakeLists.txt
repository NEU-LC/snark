# Copyright (c) 2019 The University of Sydney

install( PROGRAMS ouster-cat ouster-view
         DESTINATION ${CMAKE_INSTALL_PREFIX}/${snark_INSTALL_BIN_DIR} )

source_group( ouster-align FILES ouster-align.cpp )
add_executable( ouster-align ouster-align.cpp )
target_link_libraries( ouster-align snark_ouster )
install( TARGETS ouster-align RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

source_group( ouster-cv FILES ouster-cv.cpp )
add_executable( ouster-cv ouster-cv.cpp )
target_link_libraries( ouster-cv snark_imaging snark_ouster )
install( TARGETS ouster-cv RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

source_group( ouster-to-csv FILES ouster-to-csv.cpp )
add_executable( ouster-to-csv ouster-to-csv.cpp )
target_link_libraries( ouster-to-csv snark_ouster )
install( TARGETS ouster-to-csv RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )
