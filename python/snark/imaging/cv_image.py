#!/usr/bin/env python
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


if __name__ == '__main__':
    for i in iterator(sys.stdin):
        print >> sys.stderr, "{},{},{},{}".format(i.header['time'].item(), i.header['rows'],
                                                  i.header['cols'], i.header['type'])
        cv2.imshow('test', i.data)
        cv2.waitKey(1)