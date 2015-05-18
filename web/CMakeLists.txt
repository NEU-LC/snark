ADD_SUBDIRECTORY( applications )

IF( snark_build_web_frontend )
    SET( snark_web_frontend_dir "/var/www/html/sensors" CACHE STRING "web server directory to install snark web frontend" )
    INSTALL( DIRECTORY frontend/ DESTINATION ${snark_web_frontend_dir} USE_SOURCE_PERMISSIONS )
ENDIF( snark_build_web_frontend )

IF( snark_build_web_websocket )
    INSTALL( PROGRAMS etc/init.d/websocket DESTINATION /etc/init.d )
    INSTALL( DIRECTORY websocketd DESTINATION /usr/local/lib USE_SOURCE_PERMISSIONS )
ENDIF( snark_build_web_websocket )
