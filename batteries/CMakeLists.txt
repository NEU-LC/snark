IF( snark_build_batteries_ocean )
    ADD_SUBDIRECTORY( ocean )
ENDIF( snark_build_batteries_ocean )

IF( snark_build_batteries_braille )
    ADD_SUBDIRECTORY( braille )
ENDIF( snark_build_batteries_braille )
