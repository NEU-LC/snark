SET( dir ${SOURCE_CODE_BASE_DIR}/point_cloud/pcl/detail )
FILE( GLOB source   ${dir}/*.cpp)
FILE( GLOB includes ${dir}/*.h)
SOURCE_GROUP( ${TARGET_NAME} FILES ${source} ${includes} )

find_package(PCL 1.3 REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

ADD_EXECUTABLE( points-to-pcd points-to-pcd.cpp )
TARGET_LINK_LIBRARIES( points-to-pcd snark_math snark_point_cloud ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} ${PCL_LIBRARIES} )

ADD_EXECUTABLE( points-from-pcd points-from-pcd.cpp )
TARGET_LINK_LIBRARIES( points-from-pcd snark_math snark_point_cloud ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} ${PCL_LIBRARIES} )

if( PROFILE )
    	TARGET_LINK_LIBRARIES ( points-to-pcd profiler )
	TARGET_LINK_LIBRARIES ( points-from-pcd profiler )
endif( PROFILE )

INSTALL( TARGETS points-to-pcd
 		 points-from-pcd
         RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR}
         COMPONENT Runtime )

add_definitions (-fno-strict-aliasing)
