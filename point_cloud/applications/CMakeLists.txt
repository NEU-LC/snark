SET( dir ${SOURCE_CODE_BASE_DIR}/point_cloud/applications )
FILE( GLOB source   ${dir}/*.cpp)
FILE( GLOB includes ${dir}/*.h)
SOURCE_GROUP( ${TARGET_NAME} FILES ${source} ${includes} )

ADD_EXECUTABLE( points-align points-align.cpp )
ADD_EXECUTABLE( points-calc points-calc.cpp )
ADD_EXECUTABLE( points-detect-change points-detect-change.cpp )
ADD_EXECUTABLE( points-foreground-partitions points-foreground-partitions.cpp )
ADD_EXECUTABLE( points-rays points-rays.cpp )
ADD_EXECUTABLE( points-slice points-slice.cpp )
ADD_EXECUTABLE( points-to-centroids points-to-centroids.cpp )
ADD_EXECUTABLE( points-to-partitions points-to-partitions.cpp )
ADD_EXECUTABLE( points-to-voxels points-to-voxels.cpp )
ADD_EXECUTABLE( points-to-voxel-indices points-to-voxel-indices.cpp )
ADD_EXECUTABLE( points-track-partitions points-track-partitions.cpp )

TARGET_LINK_LIBRARIES( points-align snark_math ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} )
TARGET_LINK_LIBRARIES( points-calc snark_math snark_point_cloud ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} )
TARGET_LINK_LIBRARIES( points-detect-change snark_math ${comma_ALL_LIBRARIES} ) #profiler )
TARGET_LINK_LIBRARIES( points-foreground-partitions snark_point_cloud ${comma_ALL_LIBRARIES} tbb )
TARGET_LINK_LIBRARIES( points-rays snark_math snark_math_spherical_geometry ${comma_ALL_LIBRARIES} tbb ) #profiler )
TARGET_LINK_LIBRARIES( points-slice snark_math ${comma_ALL_LIBRARIES} )
TARGET_LINK_LIBRARIES( points-to-centroids snark_point_cloud ${comma_ALL_LIBRARIES} tbb )
TARGET_LINK_LIBRARIES( points-to-partitions snark_point_cloud ${comma_ALL_LIBRARIES} tbb )
TARGET_LINK_LIBRARIES( points-to-voxels snark_point_cloud ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} )
TARGET_LINK_LIBRARIES( points-to-voxel-indices snark_point_cloud ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} )
TARGET_LINK_LIBRARIES( points-track-partitions ${comma_ALL_LIBRARIES} )

IF( PROFILE )
    TARGET_LINK_LIBRARIES ( points-to-partitions profiler )
ENDIF( PROFILE )

INSTALL( TARGETS points-align
                 points-calc
                 points-detect-change
                 points-slice
                 points-to-partitions
                 points-foreground-partitions
                 points-rays
                 points-to-centroids
                 points-track-partitions
                 points-to-voxels
                 points-to-voxel-indices
         RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR}
         COMPONENT Runtime )

INSTALL( PROGRAMS examples/points-eigen DESTINATION ${snark_INSTALL_BIN_DIR} )

add_executable( points-mesh points-mesh.cpp )
target_link_libraries( points-mesh ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} )
install( TARGETS points-mesh RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

if( USE_CUDA )
    cuda_add_executable( points-join points-join.cpp points-join/points_join_cuda.cu ) # quick and dirty, simplistic for now
else( USE_CUDA )
    add_executable( points-join points-join.cpp )
endif( USE_CUDA )

target_link_libraries( points-join snark_math snark_math_geometry snark_point_cloud ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} ) #tbb )
install( TARGETS points-join RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

IF( snark_build_math_geometry )
    IF( NOT WIN32 )
        ADD_EXECUTABLE( points-grep points-grep.cpp )
        TARGET_LINK_LIBRARIES( points-grep snark_math_geometry snark_math ${comma_ALL_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} )
        INSTALL( TARGETS points-grep RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )
    ENDIF( NOT WIN32 )
ENDIF( snark_build_math_geometry )

ADD_SUBDIRECTORY( detail )
