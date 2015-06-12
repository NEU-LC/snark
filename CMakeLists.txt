PROJECT( "snark" )

cmake_minimum_required(VERSION 2.6)

ENABLE_TESTING()

IF (UNIX AND NOT APPLE)
  # May also fail on BSD as it assumes GNU date rather than BSD date command
	# Check if Git is installed
	find_package(Git)
	IF(GIT_FOUND)
		# Get the date of the last commit (for use by CPack)
		EXECUTE_PROCESS(
		WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
		COMMAND git log -n1 --format=%ci
		COMMAND xargs -0 date +%Y%m%dT%H%M -d
		OUTPUT_VARIABLE COMMITDATE
		OUTPUT_STRIP_TRAILING_WHITESPACE
		)
		# And also the hash.
		EXECUTE_PROCESS(
		WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
		COMMAND git rev-parse HEAD
		OUTPUT_VARIABLE COMMITHASH
		OUTPUT_STRIP_TRAILING_WHITESPACE
	)
	ELSE()
		SET ( COMMITDATE 1 )
		SET ( COMMITHASH "abc" )
	ENDIF()
ELSE() # Probably windows; gets hardcoded patch version
	SET ( COMMITDATE 1 ) 
	SET ( COMMITHASH "abc" )
ENDIF()

# ---------------------------------------------------------------------------
# set compiler and linker flags and variables

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

IF(WIN32)
  SET(CMAKE_CXX_WARNING_LEVEL 4)
  IF(CMAKE_CXX_FLAGS MATCHES "/W[0-4]")
    STRING(REGEX REPLACE "/W[0-4]" "/W4" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
  ELSE(CMAKE_CXX_FLAGS MATCHES "/W[0-4]")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
  ENDIF(CMAKE_CXX_FLAGS MATCHES "/W[0-4]")
  ADD_DEFINITIONS(/wd4512)
  ADD_DEFINITIONS(/wd4127)
  ADD_DEFINITIONS(/wd4996)
  ADD_DEFINITIONS(/wd4800)
  ADD_DEFINITIONS(/wd4244)
  SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /D \"NOMINMAX\"")
  ADD_DEFINITIONS(-D_USE_MATH_DEFINES)
  ADD_DEFINITIONS(-DWIN32_LEAN_AND_MEAN) #used to fix winsock redefinition error
ENDIF(WIN32)

IF( CMAKE_SIZEOF_VOID_P EQUAL 8 )
  SET( IS_64_BIT TRUE )
ELSE( CMAKE_SIZEOF_VOID_P EQUAL 8 )
  SET( IS_64_BIT FALSE )
ENDIF( CMAKE_SIZEOF_VOID_P EQUAL 8 )

IF(CMAKE_BUILD_TOOL MATCHES "make")
  IF(NOT CMAKE_CXX_FLAGS MATCHES "-Wall")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
  ENDIF(NOT CMAKE_CXX_FLAGS MATCHES "-Wall")
  IF(NOT CMAKE_C_FLAGS MATCHES "-Wall")
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
  ENDIF(NOT CMAKE_C_FLAGS MATCHES "-Wall")

  IF( IS_64_BIT )
    IF(NOT CMAKE_CXX_FLAGS MATCHES "-fPIC")
      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
    ENDIF(NOT CMAKE_CXX_FLAGS MATCHES "-fPIC")
    IF(NOT CMAKE_C_FLAGS MATCHES "-fPIC")
      SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
    ENDIF(NOT CMAKE_C_FLAGS MATCHES "-fPIC")
  ENDIF( IS_64_BIT )
ENDIF(CMAKE_BUILD_TOOL MATCHES "make")

SET( SOURCE_CODE_BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR} )
SET( LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib )

SET( EXECUTABLE_OUTPUT_PATH
     ${PROJECT_BINARY_DIR}/bin CACHE PATH
     "Single output directory for building all executables." )

SET( snark_LIBRARY_PROPERTIES ${snark_LIBRARY_PROPERTIES} DEBUG_POSTFIX "-d" )

IF( NOT snark_INSTALL_BIN_DIR )
  SET( snark_INSTALL_BIN_DIR bin )
ENDIF( NOT snark_INSTALL_BIN_DIR )

IF( NOT snark_INSTALL_INCLUDE_DIR )
    SET( snark_INSTALL_INCLUDE_DIR include/${PROJECT_NAME} )
ENDIF( NOT snark_INSTALL_INCLUDE_DIR )

IF( NOT snark_INSTALL_LIB_DIR)
  SET( snark_INSTALL_LIB_DIR lib )
ENDIF( NOT snark_INSTALL_LIB_DIR )

IF( NOT snark_INSTALL_PACKAGE_DIR )
  SET( snark_INSTALL_PACKAGE_DIR CMakeFiles )
ENDIF( NOT snark_INSTALL_PACKAGE_DIR )

INCLUDE_DIRECTORIES( ${SOURCE_CODE_BASE_DIR}/../ )
INCLUDE_DIRECTORIES( ${snark_INSTALL_INCLUDE_DIR}/../ )

IF( APPLE )
    SET( CMAKE_INSTALL_NAME_DIR ${CMAKE_INSTALL_PREFIX}/${snark_INSTALL_LIB_DIR} )
ENDIF( APPLE )

OPTION( BUILD_TESTS "build unit tests" OFF )
SET( snark_BUILD_TESTS ${BUILD_TESTS} )

OPTION( BUILD_APPLICATIONS "build applications" ON )
SET( snark_BUILD_APPLICATIONS ${BUILD_APPLICATIONS} )

OPTION(BUILD_SHARED_LIBS "build with shared libraries" OFF)
SET(snark_BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS})

IF( NOT BUILD_SHARED_LIBS )
    SET( Boost_USE_STATIC_LIBS ON )
ENDIF( NOT BUILD_SHARED_LIBS )
FIND_PACKAGE( Boost COMPONENTS thread filesystem system serialization date_time program_options )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${Boost_LIBRARY_DIRS} )


IF( WIN32 )
    # improve later
    #FIND_PATH( Eigen_INCLUDE_DIR NAMES Eigen/CMakeLists.txt PATHS $ENV{EIGEN_ROOT} )
    FIND_PATH( Eigen_INCLUDE_DIR NAMES signature_of_eigen3_matrix_library PATHS $ENV{EIGEN_ROOT} )
ELSEIF( APPLE )
    # actually the following line will probably work everywhere, not just APPLE (but have not tested on Windows yet)
    FIND_PATH( Eigen_INCLUDE_DIR NAMES signature_of_eigen3_matrix_library PATHS $ENV{EIGEN_ROOT} PATH_SUFFIXES eigen3 )
ELSE( WIN32 )
    #FIND_PATH( Eigen_INCLUDE_DIR Eigen/Core PATHS $ENV{EIGEN_ROOT} /usr/include/eigen3 /usr/local/include/eigen3 )
    FIND_PATH( Eigen_INCLUDE_DIR NAMES signature_of_eigen3_matrix_library PATHS $ENV{EIGEN_ROOT} /usr/include/eigen3 /usr/local/include/eigen3 )
ENDIF( WIN32 )
IF( NOT EXISTS ${Eigen_INCLUDE_DIR} )
    MESSAGE( "could not find Eigen - should be the path containing the Eigen 3 directory and /Eigen/Eigen, /Eigen/Core, etc..." )
ELSE( NOT EXISTS ${Eigen_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${Eigen_INCLUDE_DIR} )
ENDIF( NOT EXISTS ${Eigen_INCLUDE_DIR} )

IF( snark_BUILD_TESTS )
    #FIND_PACKAGE( GTest )
    #INCLUDE_DIRECTORIES(${GTEST_INCLUDE_DIRS})
    FIND_PATH( GTest_SOURCE_DIR NAMES CMakeLists.txt HINTS /usr/src/gtest DOC "source code of gtest" )
    ADD_SUBDIRECTORY( ${GTest_SOURCE_DIR} gtest )
    INCLUDE_DIRECTORIES( src ${GTest_SOURCE_DIR}/include ${GTest_SOURCE_DIR} )
    SET( GTEST_BOTH_LIBRARIES gtest gtest_main )
ENDIF( snark_BUILD_TESTS )

OPTION( snark_build_comms "build comms" ON )
OPTION( snark_build_graphics "build graphics" ON )
OPTION( snark_build_point_cloud "build point cloud" ON )
OPTION( snark_build_imaging "build imaging" ON )
OPTION( snark_build_sensors "build sensors" ON )
OPTION( snark_build_actuators "build actuators" ON )
OPTION( snark_build_battery "build battery" OFF )
OPTION( snark_build_graph "build graph" ON )
OPTION( snark_build_math_geometry "build math geometry" ON )
OPTION( snark_build_python "build python packages" OFF )
OPTION( snark_build_web "build web" OFF )
OPTION( snark_build_control "build control" OFF )

INCLUDE( CMakeDependentOption )
CMAKE_DEPENDENT_OPTION( snark_build_sensors_dc1394 "build dc1394 components" ON "snark_build_sensors" OFF )
CMAKE_DEPENDENT_OPTION( snark_build_sensors_gige "build gige components" OFF "snark_build_sensors" OFF )
CMAKE_DEPENDENT_OPTION( snark_build_sensors_flycapture "build flycapture components for pointgrey" OFF "snark_build_sensors" OFF )
CMAKE_DEPENDENT_OPTION( snark_build_sensors_sick "build sick components" ON "snark_build_sensors" OFF )
CMAKE_DEPENDENT_OPTION( snark_build_sensors_velodyne "build velodyne components" ON "snark_build_sensors" OFF )
CMAKE_DEPENDENT_OPTION( snark_build_sensors_hokuyo "build hokuyo components" ON "snark_build_sensors" OFF )
CMAKE_DEPENDENT_OPTION( snark_build_sensors_gobi "build gobi components" OFF "snark_build_sensors" OFF )

CMAKE_DEPENDENT_OPTION( snark_build_actuators_quickset "build quickset components" OFF "snark_build_actuators" OFF )
CMAKE_DEPENDENT_OPTION( snark_build_actuators_wheels "build wheels components" OFF "snark_build_actuators" OFF )
CMAKE_DEPENDENT_OPTION( snark_build_actuators_universal_robots "build universal_robots components" OFF "snark_build_actuators" OFF )

CMAKE_DEPENDENT_OPTION( snark_build_web_feeds "snark: build web feeds" ON "snark_build_web" OFF )

IF( snark_build_imaging )
    IF( WIN32 )
        SET( OpenCV_STATIC OFF )
    ENDIF( WIN32 )
    SET( OpenCV_FIND_REQUIRED )
    FIND_PACKAGE(OpenCV REQUIRED)
ENDIF( snark_build_imaging )

FIND_PACKAGE( comma PATHS ${comma_DIR} /usr/local/CMakeFiles )

if( snark_build_graphics )
    SET( snark_ALL_LIBRARIES ${snark_ALL_LIBRARIES} debug snark_graphics_qt3d-d
                             optimized snark_graphics_qt3d )
endif( snark_build_graphics )

if( snark_build_math_geometry )
    SET( snark_ALL_LIBRARIES ${snark_ALL_LIBRARIES} debug snark_geometry-d
                             optimized snark_geometry )
endif( snark_build_math_geometry )

if( snark_build_point_cloud )
    SET( snark_ALL_LIBRARIES ${snark_ALL_LIBRARIES} debug snark_point_cloud-d
                             optimized snark_point_cloud )
endif( snark_build_point_cloud )

if( snark_build_imaging )
    SET( snark_ALL_LIBRARIES ${snark_ALL_LIBRARIES} debug snark_imaging-d
                             optimized snark_imaging )
endif( snark_build_imaging )

IF( snark_build_sensors_velodyne )
    SET( snark_ALL_LIBRARIES ${snark_ALL_LIBRARIES} debug snark_velodyne-d
                             optimized snark_velodyne )
ENDIF( snark_build_sensors_velodyne )

IF( snark_build_sensors_hokuyo )
    SET( snark_ALL_LIBRARIES ${snark_ALL_LIBRARIES} debug snark_hokuyo-d
                             optimized snark_hokuyo )
ENDIF( snark_build_sensors_hokuyo )

SET( snark_ALL_LIBRARIES ${snark_ALL_LIBRARIES} debug snark_math-d
                         optimized snark_math
                         debug snark_math_spherical_geometry-d
                         optimized snark_math_spherical_geometry
                         debug snark_timing-d
                         optimized snark_timing )
IF( snark_build_actuators_quickset )
  SET( snark_ALL_LIBRARIES ${snark_ALL_LIBRARIES} debug snark_quickset_ptcr-d
                         optimized snark_quickset_ptcr )
ENDIF( snark_build_actuators_quickset )

IF( snark_build_actuators_wheels )
  SET( snark_ALL_LIBRARIES ${snark_ALL_LIBRARIES} debug snark_wheels-d
                         optimized snark_wheels )
ENDIF( snark_build_actuators_wheels )

if( snark_build_battery )
    SET( snark_ALL_LIBRARIES ${snark_ALL_LIBRARIES} debug snark_ocean-d
                             optimized snark_ocean )
endif( snark_build_battery )


IF(WIN32)
    SET( snark_ALL_EXTERNAL_LIBRARIES Ws2_32.lib )
ENDIF(WIN32)

IF( WIN32 )
    INCLUDE( CMakeFiles/FindTBB.cmake REQUIRED )
    INCLUDE_DIRECTORIES(${TBB_INCLUDE_DIRS})
    LINK_DIRECTORIES(${TBB_LIBRARY_DIRS})
ENDIF( WIN32 )

IF(UNIX AND NOT QNXNTO AND NOT APPLE)
    SET( snark_ALL_EXTERNAL_LIBRARIES rt pthread )
ENDIF(UNIX AND NOT QNXNTO AND NOT APPLE)

SET( snark_ALL_EXTERNAL_LIBRARIES ${snark_ALL_EXTERNAL_LIBRARIES} ${comma_ALL_LIBRARIES} ${comma_ALL_EXTERNAL_LIBRARIES} ${Boost_LIBRARIES} )
SET( snark_ALL_EXTERNAL_LIBRARIES ${snark_ALL_EXTERNAL_LIBRARIES} )
SET( snark_ALL_LIBRARIES ${snark_ALL_LIBRARIES} ${snark_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} ) # as gcc parses library list only once
SET( snark_EXTERNAL_INCLUDES ${Boost_INCLUDE_DIRS} ${Eigen_INCLUDE_DIR} )

CONFIGURE_FILE(
    "${CMAKE_CURRENT_SOURCE_DIR}/CMakeFiles/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

ADD_CUSTOM_TARGET(uninstall COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)

FILE( GLOB snark_PACKAGE_CMAKEFILES ${CMAKE_CURRENT_SOURCE_DIR}/CMakeFiles/*.cmake )

INCLUDE( ${CMAKE_CURRENT_SOURCE_DIR}/CMakeFiles/generate_${PROJECT_NAME}_config.cmake )

CONFIGURE_FILE( ${PROJECT_SOURCE_DIR}/CMakeFiles/use_${PROJECT_NAME}.cmake.in
                ${PROJECT_BINARY_DIR}/CMakeFiles/use_${PROJECT_NAME}.cmake COPYONLY IMMEDIATE )

SET( snark_PACKAGE_CMAKEFILES ${snark_PACKAGE_CMAKEFILES} ${PROJECT_BINARY_DIR}/CMakeFiles/use_${PROJECT_NAME}.cmake )
INSTALL( FILES ${snark_PACKAGE_CMAKEFILES}
         DESTINATION ${snark_INSTALL_PACKAGE_DIR} )

IF( snark_build_actuators )
    ADD_SUBDIRECTORY( actuators )
ENDIF( snark_build_actuators )

IF( snark_build_comms )
    ADD_SUBDIRECTORY( comms )
ENDIF( snark_build_comms )

IF( snark_build_graphics )
    ADD_SUBDIRECTORY( graphics )
ENDIF( snark_build_graphics )

IF( snark_build_point_cloud )
    ADD_SUBDIRECTORY( point_cloud )
ENDIF( snark_build_point_cloud )

ADD_SUBDIRECTORY( doc )

IF( snark_build_imaging )
    ADD_SUBDIRECTORY( tbb )
    ADD_SUBDIRECTORY( imaging )
ENDIF( snark_build_imaging )

ADD_SUBDIRECTORY( math )

IF( snark_build_sensors )
    ADD_SUBDIRECTORY( sensors )
ENDIF( snark_build_sensors )

IF( snark_build_battery )
    ADD_SUBDIRECTORY( battery )
ENDIF( snark_build_battery )

IF( snark_build_graph )
    ADD_SUBDIRECTORY( graph )
ENDIF( snark_build_graph )

add_subdirectory( audio )
add_subdirectory( geodesy )
ADD_SUBDIRECTORY( render )
add_subdirectory( terrain )
ADD_SUBDIRECTORY( timing )
ADD_SUBDIRECTORY( visiting )

IF( snark_build_python )
    ADD_SUBDIRECTORY( python )
ENDIF( snark_build_python )

IF( snark_build_web )
    ADD_SUBDIRECTORY( web )
ENDIF( snark_build_web )

IF( snark_build_control )
    ADD_SUBDIRECTORY( control )
ENDIF( snark_build_control )

# Package
#the following set up optional parameters for cpack
SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "This is a description of snark. It's great!")
SET(CPACK_PACKAGE_VENDOR "ACFR")
SET(CPACK_RESOURCE_FILE_LICENSE "${PROJECT_SOURCE_DIR}/COPYING")
SET(CPACK_PACKAGE_VERSION_MAJOR "1")
SET(CPACK_PACKAGE_VERSION_MINOR ${COMMITDATE})
SET(CPACK_PACKAGE_VERSION_PATCH ${COMMITHASH})
SET(CPACK_PACKAGE_CONTACT "t.teo@acfr.usyd.edu.au")
SET(CPACK_RPM_PACKAGE_REQUIRES "comma")

# Additional Win32 handling for CPack.
IF( WIN32 )
    if( snark_build_imaging )
        file( GLOB OpenCV_DLLS ${OpenCV_LIB_PATH}/../bin/*.dll ) #from opencv version 2.4
    endif( snark_build_imaging )

    if( snark_build_graphics )
        get_filename_component( QT_BIN_DIR ${QT_QMAKE_EXECUTABLE} PATH )
        file( GLOB QT_DLLS ${QT_BIN_DIR}/*.dll )
    endif( snark_build_graphics )

    SET(snark_PACKAGE_DLLS ${TBB_DLL_DIR}/tbb.dll ${OpenCV_DLLS} ${QT_DLLS})
    
    INSTALL(FILES ${snark_PACKAGE_DLLS} DESTINATION bin )

    #this line finds the appropriate compiler dlls, e.g. Visual Studio DLLS in windows. Maybe it would find other dlls if a different build tool were used.
    INCLUDE( InstallRequiredSystemLibraries )
ENDIF( WIN32 )

INCLUDE( CPack )
# End Packaging
