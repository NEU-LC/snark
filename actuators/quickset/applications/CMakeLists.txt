source_group( quickset-pantilt-control FILES quickset-pantilt-control.cpp )
add_executable( quickset-pantilt-control quickset-pantilt-control.cpp )
target_link_libraries( quickset-pantilt-control snark_quickset_ptcr ${snark_ALL_EXTERNAL_LIBRARIES} ${Boost_ATOMIC_LIBRARY} )

source_group( quickset-pantilt-from-console FILES quickset-pantilt-from-console.cpp )
add_executable( quickset-pantilt-from-console quickset-pantilt-from-console.cpp )
target_link_libraries( quickset-pantilt-from-console ${snark_ALL_EXTERNAL_LIBRARIES} ${Boost_ATOMIC_LIBRARY})

install( TARGETS quickset-pantilt-from-console quickset-pantilt-control
         RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR}
         COMPONENT Runtime )
