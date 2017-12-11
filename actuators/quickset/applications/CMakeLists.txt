source_group( quickset-pantilt-control FILES quickset-pantilt-control.cpp )
add_executable( quickset-pantilt-control quickset-pantilt-control.cpp )
target_link_libraries( quickset-pantilt-control snark_quickset_ptcr ${snark_ALL_EXTERNAL_LIBRARIES} ${Boost_ATOMIC_LIBRARY} )

install( TARGETS quickset-pantilt-control RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )
install( PROGRAMS quickset-pantilt-web-control DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )
