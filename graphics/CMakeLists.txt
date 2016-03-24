SET( PROJECT "graphics" )
SET( TARGET_NAME snark_${PROJECT} )

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

IF(DEFINED QT_VERSION)
    SET(snark_QT_DEFAULT_VERSION ${QT_VERSION})
ELSE(DEFINED QT_VERSION)
    # Use qmake to find Qt version                 
    EXECUTE_PROCESS( COMMAND qmake -v COMMAND grep "Qt version" COMMAND sed "s/.* \\([0-9]\\.[0-9]\\.[0-9]\\) .*/\\1/" 
                     OUTPUT_VARIABLE snark_QT_DEFAULT_VERSION 
                     OUTPUT_STRIP_TRAILING_WHITESPACE)
                     
    IF(NOT(DEFINED snark_QT_DEFAULT_VERSION) OR snark_QT_DEFAULT_VERSION EQUAL "")
        MESSAGE(FATAL_ERROR "Please set QT_VERSION to the Qt version number")
    ENDIF(NOT(DEFINED snark_QT_DEFAULT_VERSION) OR snark_QT_DEFAULT_VERSION EQUAL "")
ENDIF(DEFINED QT_VERSION)

set( snark_QT_VERSION "${snark_QT_DEFAULT_VERSION}" CACHE STRING "Qt version" )

IF(snark_QT_VERSION MATCHES "^[0-9]+\\.[0-9]+")
    STRING( REPLACE "." ";" QT_VERSION_LIST ${snark_QT_VERSION} )
    LIST( GET QT_VERSION_LIST 0 QT_VERSION_MAJOR )
    LIST( GET QT_VERSION_LIST 1 QT_VERSION_MINOR )
ELSEIF(snark_QT_VERSION MATCHES "^[0-9]+$")
    SET(QT_VERSION_MAJOR ${snark_QT_VERSION})
    SET(QT_VERSION_MINOR "0")
ELSE(snark_QT_VERSION MATCHES "^[0-9]+\\.[0-9]+")
    MESSAGE( FATAL_ERROR "Malformed QT version: ${snark_QT_VERSION}" )
ENDIF(snark_QT_VERSION MATCHES "^[0-9]+\\.[0-9]+")

IF( QT_VERSION_MAJOR LESS 4 OR QT_VERSION_MAJOR GREATER 5 )
    MESSAGE( FATAL_ERROR "Qt version 4 or 5 required" )
ENDIF( QT_VERSION_MAJOR LESS 4 OR QT_VERSION_MAJOR GREATER 5 )

ADD_DEFINITIONS(-DQT_VERSION_MAJOR=${QT_VERSION_MAJOR} -DQT_VERSION_MINOR=${QT_VERSION_MINOR})
    
IF( QT_VERSION_MAJOR EQUAL 4 )
    FIND_PACKAGE( Qt4 COMPONENTS QtCore QtGui QtOpenGl QtXml REQUIRED )
    INCLUDE( ${QT_USE_FILE} )
    FIND_LIBRARY( Qt3D_LIB NAMES Qt3D REQUIRED )
ELSEIF( QT_VERSION_MAJOR EQUAL 5 )
    IF( QT_VERSION_MINOR LESS 5 )
        FIND_PACKAGE( Qt5 COMPONENTS Core Gui OpenGL Xml 3D 3DQuick REQUIRED )
    ELSE( QT_VERSION_MINOR LESS 5 )
        MESSAGE( FATAL_ERROR "Qt5.5+ 3D support not yet implemented" )
    ENDIF( QT_VERSION_MINOR LESS 5 )
ENDIF( QT_VERSION_MAJOR EQUAL 4 )

SET( dir ${SOURCE_CODE_BASE_DIR}/${PROJECT} )
FILE( GLOB source   ${dir}/*.cpp)
FILE( GLOB includes ${dir}/*.h )

SOURCE_GROUP( ${TARGET_NAME} FILES ${source} ${includes} )

IF( snark_BUILD_APPLICATIONS )
    ADD_SUBDIRECTORY( applications )
ENDIF( snark_BUILD_APPLICATIONS )

ADD_SUBDIRECTORY( qt3d )
