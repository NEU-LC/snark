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

    # todo fix handling of end of file or stdin

    # first time, read header and calculate data types
    header = np.fromfile(file, image.header_dtype, 1)[0]
    type_string, channels = cv_types.cv2string(header['type'])

    # get the image data
    data_dtype = np.dtype((np.dtype(type_string), (header['rows'], header['cols'], channels)))
    data = np.fromfile(file, data_dtype, 1)[0]

    while True:
        yield image(header, data)
        header = np.fromfile(file, image.header_dtype, 1)[0]
        data = np.fromfile(file, data_dtype, 1)[0]


if __name__ == '__main__':
    file = sys.stdin
    images = iterator(file)

    while True:
        image = images.next()
        print >> sys.stderr, "{},{},{},{}".format(image.header['time'].item(), image.header['rows'],
                                                  image.header['cols'], image.header['type'])
        cv2.imshow('test', image.data)
        cv2.waitKey(1)