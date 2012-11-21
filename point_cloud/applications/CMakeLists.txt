SET( dir ${SOURCE_CODE_BASE_DIR}/point_cloud/applications )
FILE( GLOB source   ${dir}/*.cpp)
FILE( GLOB includes ${dir}/*.h)
SOURCE_GROUP( ${TARGET_NAME} FILES ${source} ${includes} )

ADD_EXECUTABLE( points-to-partitions points-to-partitions.cpp )
ADD_EXECUTABLE( points-track-partitions points-track-partitions.cpp )
ADD_EXECUTABLE( points-to-voxels points-to-voxels.cpp )
ADD_EXECUTABLE( points-to-voxel-indices points-to-voxel-indices.cpp )

TARGET_LINK_LIBRARIES ( points-to-partitions snark_point_cloud ${comma_ALL_LIBRARIES} tbb )
TARGET_LINK_LIBRARIES ( points-track-partitions ${comma_ALL_LIBRARIES} )
TARGET_LINK_LIBRARIES ( points-to-voxels snark_point_cloud ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} )
TARGET_LINK_LIBRARIES ( points-to-voxel-indices snark_point_cloud ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} )

if( PROFILE )
    TARGET_LINK_LIBRARIES ( points-to-partitions profiler )
endif( PROFILE )

INSTALL( TARGETS points-to-partitions
                 points-track-partitions
                 points-to-voxels
                 points-to-voxel-indices
         RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR}
         COMPONENT Runtime )
