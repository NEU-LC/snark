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
    add_subdirectory( ${GTest_SOURCE_DIR} gtest )
    INCLUDE_DIRECTORIES( src ${GTest_SOURCE_DIR}/include ${GTest_SOURCE_DIR} )
    SET( GTEST_BOTH_LIBRARIES gtest gtest_main )
	
#TT: Install tests to dir. This allows them to be packed by CPack.
SET ( snark_CPP_TESTS_INSTALL_DIR bin CACHE PATH "Install directory for cpp unit tests" )
    #SET ( snark_CPP_TESTS_INSTALL_DIR share/test/${PROJECT_NAME} )
ENDIF( snark_BUILD_TESTS )

OPTION( snark_build_actuators "build actuators" ON )
OPTION( snark_build_batteries "build batteries" OFF )
OPTION( snark_build_comms "build comms" ON )
OPTION( snark_build_control "build control" OFF )
OPTION( snark_build_graph "build graph" ON )
OPTION( snark_build_graphics "build graphics" ON )
OPTION( snark_build_imaging "build imaging" ON )
OPTION( snark_build_math_geometry "build math geometry" ON )
OPTION( snark_build_navigation "build navigation" OFF )
OPTION( snark_build_point_cloud "build point cloud" ON )
OPTION( snark_build_point_cloud_pcl "build points cloud pcl" OFF )
OPTION( snark_build_sensors "build sensors" ON )
OPTION( snark_build_python "build python packages" OFF )
OPTION( snark_build_web "build web" OFF )
OPTION( snark_build_terrain "build terrain" OFF )
option( snark_build_math_fft "build math-fft" OFF )

INCLUDE( CMakeDependentOption )
CMAKE_DEPENDENT_OPTION( ADD_PYTHON_PACKAGES_TO_RPM "add snark python packages to RPM" OFF "snark_build_python" ON )

CMAKE_DEPENDENT_OPTION( snark_build_sensors_dc1394 "build dc1394 components" ON "snark_build_sensors" OFF )
CMAKE_DEPENDENT_OPTION( snark_build_sensors_gige "build gige components" OFF "snark_build_sensors" OFF )
CMAKE_DEPENDENT_OPTION( snark_build_sensors_flycapture "build flycapture components for pointgrey" OFF "snark_build_sensors" OFF )
CMAKE_DEPENDENT_OPTION( snark_build_sensors_sick "build sick components" ON "snark_build_sensors" OFF )
CMAKE_DEPENDENT_OPTION( snark_build_sensors_velodyne "build velodyne components" ON "snark_build_sensors" OFF )
CMAKE_DEPENDENT_OPTION( snark_build_sensors_hokuyo "build hokuyo components" ON "snark_build_sensors" OFF )
CMAKE_DEPENDENT_OPTION( snark_build_sensors_gobi "build gobi components" OFF "snark_build_sensors" OFF )
CMAKE_DEPENDENT_OPTION( snark_build_sensors_jai "build jai components" OFF "snark_build_sensors" OFF )
CMAKE_DEPENDENT_OPTION( snark_build_sensors_basler "build basler components" OFF "snark_build_sensors" OFF )
CMAKE_DEPENDENT_OPTION( snark_build_sensors_asd "build asd components" OFF "snark_build_sensors" OFF )

CMAKE_DEPENDENT_OPTION( snark_build_actuators_quickset "build quickset components" OFF "snark_build_actuators" OFF )
CMAKE_DEPENDENT_OPTION( snark_build_actuators_wheels "build wheels components" OFF "snark_build_actuators" OFF )
CMAKE_DEPENDENT_OPTION( snark_build_actuators_universal_robots "build universal_robots components" OFF "snark_build_actuators" OFF )

CMAKE_DEPENDENT_OPTION( snark_build_web_feeds "snark: build web feeds" ON "snark_build_web" OFF )

CMAKE_DEPENDENT_OPTION( snark_build_batteries_ocean "snark: build ocean battery" OFF "snark_build_batteries" OFF )
CMAKE_DEPENDENT_OPTION( snark_build_batteries_braille "snark: build braille battery" OFF "snark_build_batteries" OFF )

SET( snark_system_manager "auto" CACHE STRING "the system and service manager of the operating system, options are: auto init systemd" )
IF( snark_system_manager STREQUAL "auto" )
    EXECUTE_PROCESS( COMMAND cat /proc/1/comm
                     OUTPUT_VARIABLE snark_system_manager
                     OUTPUT_STRIP_TRAILING_WHITESPACE
                     ERROR_QUIET )
    MESSAGE( STATUS "auto detected system manager: ${snark_system_manager}" )
ENDIF( snark_system_manager STREQUAL "auto" )

IF( snark_build_imaging )
    IF( WIN32 )
        SET( OpenCV_STATIC OFF )
    ENDIF( WIN32 )
    SET( OpenCV_FIND_REQUIRED )
    FIND_PACKAGE(OpenCV REQUIRED)
######## start
#    message( "OpenCV_VERSION: ${OpenCV_VERSION}")
#     "${OpenCV_VERSION}" STREQUAL "2.4.11"
    if( "${OpenCV_VERSION}"  STRGREATER "2.4.10" )
#        message( "greater than 2.4.10")
        EXECUTE_PROCESS( COMMAND qmake -v COMMAND grep "Qt version" COMMAND sed "s/.* \\([0-9]\\)\\.[0-9]\\.[0-9] .*/\\1/" OUTPUT_VARIABLE snark_QT_DEFAULT_VERSION )
        IF( snark_QT_DEFAULT_VERSION EQUAL "5" )
            SET( snark_QT_VERSION "5" CACHE STRING "Qt version" )
        ELSE( snark_QT_DEFAULT_VERSION EQUAL "5" )
            IF( snark_QT_DEFAULT_VERSION EQUAL "4" )
                SET( snark_QT_VERSION "4" CACHE STRING "Qt version" )
            ENDIF( snark_QT_DEFAULT_VERSION EQUAL "4" )
        ENDIF( snark_QT_DEFAULT_VERSION EQUAL "5" ) 

        IF( snark_QT_VERSION EQUAL "5" )
            FIND_PACKAGE( Qt5 COMPONENTS Core Gui OpenGL Xml 3D 3DQuick REQUIRED )
        ELSE ( snark_QT_VERSION EQUAL "5" )
            IF( snark_QT_VERSION EQUAL "4" )
                FIND_PACKAGE( Qt4 COMPONENTS QtCore QtGui QtOpenGl QtXml REQUIRED )
                INCLUDE( ${QT_USE_FILE} )
                FIND_LIBRARY( Qt3D_LIB NAMES Qt3D REQUIRED )
            ELSE ( snark_QT_VERSION EQUAL "4" )
                MESSAGE( FATAL_ERROR "please specify 4 or 5 as QT_VERSION" )
            ENDIF ( snark_QT_VERSION EQUAL "4" )
        ENDIF( snark_QT_VERSION EQUAL "5" )
    endif()
######### end
    
    
ENDIF( snark_build_imaging )

FIND_PACKAGE( comma PATHS ${comma_DIR} /usr/local/CMakeFiles )

if( snark_build_graphics )
    SET( snark_ALL_LIBRARIES ${snark_ALL_LIBRARIES} debug snark_graphics_qt3d-d
                             optimized snark_graphics_qt3d )
endif( snark_build_graphics )

if( snark_build_math_geometry )
    SET( snark_ALL_LIBRARIES ${snark_ALL_LIBRARIES} debug snark_math_geometry-d
                             optimized snark_math_geometry )
endif( snark_build_math_geometry )

if( snark_build_point_cloud )
    SET( snark_ALL_LIBRARIES ${snark_ALL_LIBRARIES} debug snark_point_cloud-d
                             optimized snark_point_cloud )
endif( snark_build_point_cloud )

if( snark_build_imaging )
    SET( snark_ALL_LIBRARIES ${snark_ALL_LIBRARIES} debug snark_imaging-d
                             optimized snark_imaging )
endif( snark_build_imaging )

# todo: uncomment, once implemented
# if( snark_build_navigation )
#     set( snark_ALL_LIBRARIES ${snark_ALL_LIBRARIES} debug snark_navigation_trimble-d
#                              optimized snark_navigation_trimble )
# endif( snark_build_navigation )

IF( snark_build_sensors_velodyne )
    SET( snark_ALL_LIBRARIES ${snark_ALL_LIBRARIES} debug snark_velodyne-d
                             optimized snark_velodyne )
ENDIF( snark_build_sensors_velodyne )

IF( snark_build_sensors_hokuyo )
    SET( snark_ALL_LIBRARIES ${snark_ALL_LIBRARIES} debug snark_hokuyo-d
                             optimized snark_hokuyo )
ENDIF( snark_build_sensors_hokuyo )

IF( snark_build_sensors_asd )
    SET( snark_ALL_LIBRARIES ${snark_ALL_LIBRARIES} debug snark_asd-d
                             optimized snark_asd )
ENDIF( snark_build_sensors_asd )

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

if( snark_build_batteries_ocean )
    SET( snark_ALL_LIBRARIES ${snark_ALL_LIBRARIES} debug snark_ocean-d
                             optimized snark_ocean )
endif( snark_build_batteries_ocean )

if( snark_build_control )
    SET( snark_ALL_LIBRARIES ${snark_ALL_LIBRARIES} debug snark_control-d
                             optimized snark_control )
endif( snark_build_control )


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

add_subdirectory( actuators )

IF( snark_build_comms )
    add_subdirectory( comms )
ENDIF( snark_build_comms )

IF( snark_build_graphics )
    add_subdirectory( graphics )
ENDIF( snark_build_graphics )

IF( snark_build_point_cloud )
    add_subdirectory( point_cloud )
ENDIF( snark_build_point_cloud )

add_subdirectory( doc )

IF( snark_build_imaging )
    add_subdirectory( tbb )
    add_subdirectory( imaging )
ENDIF( snark_build_imaging )

if( snark_build_navigation )
    add_subdirectory( navigation )
endif( snark_build_navigation )

add_subdirectory( math )

IF( snark_build_sensors )
    add_subdirectory( sensors )
ENDIF( snark_build_sensors )

IF( snark_build_batteries )
    add_subdirectory( batteries )
ENDIF( snark_build_batteries )

IF( snark_build_graph )
    add_subdirectory( graph )
ENDIF( snark_build_graph )

add_subdirectory( audio )
add_subdirectory( geodesy )
add_subdirectory( render )
add_subdirectory( timing )
add_subdirectory( visiting )
add_subdirectory( control )

IF( snark_build_terrain )
    add_subdirectory( terrain )
ENDIF( snark_build_terrain )

add_subdirectory( etc/bash_completion.d )

add_subdirectory( python )

IF( snark_build_web )
    add_subdirectory( web )
ENDIF( snark_build_web )

# Package
#the following set up optional parameters for cpack
SET(CPACK_PACKAGE_NAME "snark")
SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Snark contains various libraries and utilities for interacting with robots and working with sensor data.")
SET(CPACK_PACKAGE_VENDOR "ACFR")
SET(CPACK_RESOURCE_FILE_LICENSE "${PROJECT_SOURCE_DIR}/COPYING")
SET(CPACK_PACKAGE_VERSION_MAJOR "1")
SET(CPACK_PACKAGE_VERSION_MINOR ${COMMITDATE})
SET(CPACK_PACKAGE_VERSION_PATCH ${COMMITHASH})
SET(CPACK_PACKAGE_CONTACT "t.teo@acfr.usyd.edu.au")


IF( snark_build_python AND ADD_PYTHON_PACKAGES_TO_RPM )
    SET(CPACK_RPM_PACKAGE_REQUIRES "comma python numpy python-opencv")
    SET(CPACK_RPM_POST_INSTALL_SCRIPT_FILE "${CMAKE_SOURCE_DIR}/python/snark-rpm-post-install.sh" )
    SET(CPACK_RPM_PRE_INSTALL_SCRIPT_FILE "${CMAKE_SOURCE_DIR}/python/snark-rpm-pre-install.sh" )
	# Prevents /tmp from permission change when installing RPM package
	SET(CPACK_RPM_EXCLUDE_FROM_AUTO_FILELIST_ADDITION "/tmp" )
ELSE( snark_build_python AND ADD_PYTHON_PACKAGES_TO_RPM )
    SET(CPACK_RPM_PACKAGE_REQUIRES "comma")
ENDIF( snark_build_python AND ADD_PYTHON_PACKAGES_TO_RPM )

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
