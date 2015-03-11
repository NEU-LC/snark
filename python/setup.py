#!/usr/bin/env python

from distutils.core import setup

setup(
        name                = 'snark',
        version             = '1.0',
        description         = 'snark python utilties',
        url                 = 'https://github.com/acfr/snark',
        license             = 'BSD 3-Clause',
        packages            = [ 'snark', 'snark.imaging' ],
        scripts             = [ 'snark/imaging/applications/image-intensity' ],
     )
