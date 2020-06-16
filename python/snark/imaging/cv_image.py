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
'''
Provides file reader for cv-cat style binary data
'''

from __future__ import print_function

import sys

import cv2
import numpy as np
from snark.imaging import cv_types

__author__ = 'j.underwood'


try:
    STDOUT = sys.stdout.buffer
    STDIN = sys.stdin.buffer
except AttributeError:
    STDOUT = sys.stdout
    STDIN = sys.stdin


def safe_read(file, dtype, count):
    """Compatible with Python 2 and 3; duplicates read data for short period."""
    try:
        buf = file.read(dtype.itemsize)
        return np.frombuffer(buf, dtype, count)
    except ValueError:
        raise EOFError()


class image():
    header_dtype = np.dtype([('time', 'datetime64[us]'),
                             ('rows', np.uint32),
                             ('cols', np.uint32),
                             ('type', np.uint32)])

    def __init__(self, header=None, data=None):
        self.header = header
        self.data = data

    def validate(self):
        if self.header is not None:
            type_string, channels = cv_types.cv2string(self.header['type'])
            if self.data.dtype.name != type_string:
                raise ValueError("cv_image dtype mismatch ({}, expected {})".format(self.data.dtype.name, type_string))
            expected_shape = (self.header['rows'], self.header['cols'], channels)
            if self.data.shape != expected_shape:
                raise ValueError("cv_image shape mismatch ({}, expected {})".format(self.data.shape, expected_shape))

    def write(self, file=STDOUT, flush=False):
        self.validate()
        if self.header is not None:
            file.write(self.header.tobytes())
        if self.data is not None:
            file.write(self.data.tobytes())
        if flush:
            file.flush()

    @staticmethod
    def read(file, frame_dtype=None):
        """If frame_dtype is None then assume it has a header and read it"""
        header = None
        frame = None
        if frame_dtype is None:
            header = safe_read(file, image.header_dtype, 1)[0]
            type_string, channels = cv_types.cv2string(header['type'])
            frame_dtype = np.dtype((np.dtype(type_string), (header['rows'], header['cols'], channels)))
        frame = safe_read(file, frame_dtype, 1)[0]
        return image(header, frame)


def iterator(file, rows=None, cols=None, dtype=None):
    """
    read binary cv-cat data from file object in the form t,rows,cols,type,data, t,3ui,data.
    if rows, cols and dtype provided, assume there is no header in the stream.
    dtype is specified by: [num_channels]{uint8, int8, uint16, int16, int32, float32, float64}
    e.g.: iterator(sys.stdin, rows=512, cols=512, dtype='3uint8')
    """
    assert (rows is None and cols is None and dtype is None) or (rows and cols and dtype), \
        "rows, cols, dtype should be all None or all not None"
    frame_dtype = None
    if rows is not None:
        try:
            channels = int(dtype[0])
            dtype = dtype[1:]
        except ValueError:
            channels = 1
        frame_dtype = np.dtype((np.dtype(dtype), (rows, cols, channels)))
    try:
        while True: yield image.read(file, frame_dtype)
    except EOFError:
        return


def zip_images(*input_specs):
    """Read stream(s) given path and header input specifications; if no header specification
    is given, then assume it can be read from the stream.
        input_spec=<path>[;rows=<rows>;cols=<cols>;type=<dtype>] (dtype as in cv_image.iterator)"""
    iterators = []
    files = []
    for input_spec in input_specs:
        parts = input_spec.split(';')
        header = dict([tuple(part.split('=')) for part in parts if '=' in part])
        rows = int(header['rows']) if 'rows' in header else None
        cols = int(header['cols']) if 'cols' in header else None
        dtype = header['type'] if 'type' in header else None
        files.append(open(parts[0], 'rb'))
        iterators.append(iterator(files[-1], rows=rows, cols=cols, dtype=dtype))
    try:
        while True:
            yield tuple([next(stream_iter) for stream_iter in iterators])
    except StopIteration:
        return


def write(frame, flush=False, out_file=STDOUT):
    """Method for backwards compatibility, use frame.write() instead."""
    frame.write(out_file, flush=False)


if __name__ == '__main__':
    for i in iterator(STDIN):
        print("{},{},{},{}".format(
            i.header['time'].item(),
            i.header['rows'], i.header['cols'], i.header['type']), file=sys.stderr)
        cv2.imshow('test', i.data)
        cv2.waitKey(1)
