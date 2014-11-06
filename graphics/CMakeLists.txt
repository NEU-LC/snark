SET( PROJECT "graphics" )

IF( WIN32 OR APPLE )
    FIND_PACKAGE( OpenGL )
ELSE( WIN32 OR APPLE )
    FIND_LIBRARY( OPENGL_LIBRARY NAMES GL REQUIRED )
ENDIF( WIN32 OR APPLE )

# If Qt5 is found, provide an option to select which Qt to use
# Note: if both Qt4 and Qt5 include directories end up in the include path
#   (e.g. if the unused one is in /usr/local/include alongside another dependency like boost)
#   then you'll get conflicts between them as many header files have the same names in both.
# So if you need two different Qt installations, keep them out of the 'standard' places.

# if both qt4 and qt5 are installed, the following find_package line forces
# make to use Qt5 version of moc even if snark_use_Qt5 is set to OFF
#FIND_PACKAGE( Qt5 COMPONENTS Core QUIET )
#CMAKE_DEPENDENT_OPTION( snark_use_Qt5 "build against Qt5 (off means Qt4)" ON "snark_build_graphics; Qt5_FOUND" OFF )

#IF(snark_use_Qt5)
#    FIND_PACKAGE( Qt5 COMPONENTS Core Gui OpenGL Xml 3D 3DQuick REQUIRED )
#ELSE(snark_use_Qt5)
#    FIND_PACKAGE( Qt4 COMPONENTS QtCore QtGui QtOpenGl QtXml REQUIRED )
#    INCLUDE( ${QT_USE_FILE} )
#    FIND_LIBRARY( Qt3D_LIB NAMES Qt3D REQUIRED )
#ENDIF(snark_use_Qt5)

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

SET( dir ${SOURCE_CODE_BASE_DIR}/${PROJECT} )
FILE( GLOB includes ${dir}/*.h )
INSTALL( FILES ${includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/${PROJECT} )

IF( snark_BUILD_APPLICATIONS )
    ADD_SUBDIRECTORY( applications )
ENDIF( snark_BUILD_APPLICATIONS )

ADD_SUBDIRECTORY( qt3d )
