SOURCE_GROUP( quickset-pantilt-control FILES quickset-pantilt-control.cpp )
ADD_EXECUTABLE( quickset-pantilt-control quickset-pantilt-control.cpp )
TARGET_LINK_LIBRARIES( quickset-pantilt-control snark_quickset_ptcr ${snark_ALL_EXTERNAL_LIBRARIES} ${Boost_ATOMIC_LIBRARY} )

SOURCE_GROUP( quickset-pantilt-from-console FILES quickset-pantilt-from-console.cpp )
ADD_EXECUTABLE( quickset-pantilt-from-console quickset-pantilt-from-console.cpp )
TARGET_LINK_LIBRARIES( quickset-pantilt-from-console ${snark_ALL_EXTERNAL_LIBRARIES} ${Boost_ATOMIC_LIBRARY})

INSTALL( TARGETS quickset-pantilt-from-console quickset-pantilt-control
         RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR}
         COMPONENT Runtime )
