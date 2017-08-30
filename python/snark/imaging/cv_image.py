#!/usr/bin/env python

# This file is part of comma, a generic and flexible library
# Copyright (c) 2011 The University of Sydney
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the University of Sydney nor the
#    names of its contributors may be used to endorse or promote products
#    derived from this software without specific prior written permission.
#
# NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
# GRANTED BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
# HOLDERS AND CONTRIBUTORS \"AS IS\" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
# BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
# IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

__author__ = 'j.underwood'

import sys
import cv2
import numpy as np
from snark.imaging import cv_types

'''
Provides file reader for cv-cat style binary data
'''


class image():
    header_dtype = np.dtype([('time', 'datetime64[us]'),
                             ('rows', np.uint32),
                             ('cols', np.uint32),
                             ('type', np.uint32)])

    def __init__(self, header, data):
        self.header = header
        self.data = data


def iterator(file):
    """
    read binary cv-cat data from file object in the form t,rows,cols,type,data, t,3ui,data
    """

    # first time, read header and calculate data types
    header = np.fromfile(file, image.header_dtype, 1)
    if header.shape[0]==0: return
    type_string, channels = cv_types.cv2string(header[0]['type'])
    
    # get the image data
    data_dtype = np.dtype((np.dtype(type_string), (header[0]['rows'], header[0]['cols'], channels)))
    data = np.fromfile(file, data_dtype, 1)

    while (header.shape[0]==1 & data.shape[0]==1):
        yield image(header[0], data[0])
        header = np.fromfile(file, image.header_dtype, 1)
        data = np.fromfile(file, data_dtype, 1)

def write( image, flush=False, out_file=sys.stdout ):
    image.header.tofile( out_file )
    image.data.tofile( out_file )
    if flush: out_file.flush()

if __name__ == '__main__':
    for i in iterator(sys.stdin):
        print >> sys.stderr, "{},{},{},{}".format(i.header['time'].item(), i.header['rows'],
                                                  i.header['cols'], i.header['type'])
        cv2.imshow('test', i.data)
        cv2.waitKey(1)
