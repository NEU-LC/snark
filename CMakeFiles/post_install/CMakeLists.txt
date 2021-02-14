if( BUILD_SHARED_LIBS )
    install( CODE "message( \"ldconfig: running...\" )" )
    install( CODE "execute_process( COMMAND /sbin/ldconfig )" )
    install( CODE "message( \"ldconfig: done\" )" )
endif( BUILD_SHARED_LIBS )

