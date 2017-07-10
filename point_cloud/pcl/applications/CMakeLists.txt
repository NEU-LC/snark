set( dir ${SOURCE_CODE_BASE_DIR}/point_cloud/pcl/detail )
file( GLOB source   ${dir}/*.cpp)
file( GLOB includes ${dir}/*.h)
source_group( ${TARGET_NAME} FILES ${source} ${includes} )

find_package( PCL 1.3 REQUIRED )
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

add_executable( points-to-pcd points-to-pcd.cpp )
target_link_libraries( points-to-pcd snark_math snark_point_cloud ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} ${PCL_LIBRARIES} )

add_executable( points-from-pcd points-from-pcd.cpp )
target_link_libraries( points-from-pcd snark_math snark_point_cloud ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} ${PCL_LIBRARIES} )

add_executable( points-pcl-calc points-pcl-calc.cpp )
# target_link_libraries( points-pcl-calc ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} pcl_segmentation pcl_kdtree pcl_common )
target_link_libraries( points-pcl-calc ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} ${PCL_LIBRARIES} )

if( PROFILE )
    target_link_libraries( points-to-pcd profiler )
    target_link_libraries( points-from-pcd profiler )
endif( PROFILE )

install( TARGETS points-to-pcd points-from-pcd points-pcl-calc RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

add_definitions( -fno-strict-aliasing )
