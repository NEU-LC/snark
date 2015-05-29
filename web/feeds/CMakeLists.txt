ADD_SUBDIRECTORY( applications )

SET( snark_web_feeds_dir "/var/www/html/feeds" CACHE STRING "web server directory to install snark web feeds frontend" )
INSTALL( DIRECTORY frontend/ DESTINATION ${snark_web_feeds_dir} USE_SOURCE_PERMISSIONS )
FILE( GLOB init_scripts etc/init.d/* )
INSTALL( PROGRAMS ${init_scripts} DESTINATION /etc/init.d )
INSTALL( DIRECTORY websocketd DESTINATION /usr/local/lib USE_SOURCE_PERMISSIONS )
