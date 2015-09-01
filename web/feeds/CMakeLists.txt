ADD_SUBDIRECTORY( applications )

SET( snark_web_feeds_dir "/var/www/html/feeds" CACHE STRING "web server directory to install snark web feeds frontend" )
INSTALL( DIRECTORY frontend/ DESTINATION ${snark_web_feeds_dir} USE_SOURCE_PERMISSIONS )
INSTALL( CODE "EXECUTE_PROCESS( COMMAND mkdir -p ${snark_web_feeds_dir}/config )" )

INSTALL( DIRECTORY websocketd DESTINATION /usr/local/lib USE_SOURCE_PERMISSIONS )

IF( snark_system_manager STREQUAL "init" )
    FILE( GLOB init_scripts etc/init.d/* )
    INSTALL( PROGRAMS ${init_scripts} DESTINATION /etc/init.d )
ELSEIF( snark_system_manager STREQUAL "systemd" )
    FILE( GLOB systemd_unit_files systemd/* )
    INSTALL( FILES ${systemd_unit_files} DESTINATION /usr/local/lib/systemd/system )
ELSE()
    MESSAGE( WARNING "unsupported system manager: '${snark_system_manager}'" )
ENDIF()
