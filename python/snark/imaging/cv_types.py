#!/usr/bin/env python
__author__ = 'j.underwood'

'''
Provides conversion functions between opencv and type strings, compatible with np dtype
'''

import cv2


def cv2string(cv):
    """
    convert from cv typdef to type string and number of channels
    """
    conversion = {
            cv2.CV_8UC1: ('uint8', 1),
            cv2.CV_8UC2: ('uint8', 2),
            cv2.CV_8UC3: ('uint8', 3),
            cv2.CV_8UC4: ('uint8', 4),

            cv2.CV_8SC1: ('int8', 1),
            cv2.CV_8SC2: ('int8', 2),
            cv2.CV_8SC3: ('int8', 3),
            cv2.CV_8SC4: ('int8', 4),

            cv2.CV_16UC1: ('uint16', 1),
            cv2.CV_16UC2: ('uint16', 2),
            cv2.CV_16UC3: ('uint16', 3),
            cv2.CV_16UC4: ('uint16', 4),

            cv2.CV_16SC1: ('int16', 1),
            cv2.CV_16SC2: ('int16', 2),
            cv2.CV_16SC3: ('int16', 3),
            cv2.CV_16SC4: ('int16', 4),

            cv2.CV_32SC1: ('int32', 1),
            cv2.CV_32SC2: ('int32', 2),
            cv2.CV_32SC3: ('int32', 3),
            cv2.CV_32SC4: ('int32', 4),

            cv2.CV_32FC1: ('float32', 1),
            cv2.CV_32FC2: ('float32', 2),
            cv2.CV_32FC3: ('float32', 3),
            cv2.CV_32FC4: ('float32', 4),

            cv2.CV_64FC1: ('float64', 1),
            cv2.CV_64FC2: ('float64', 2),
            cv2.CV_64FC3: ('float64', 3),
            cv2.CV_64FC4: ('float64', 4)
    }
    t = conversion[cv]
    return t[0], t[1]


def string2cv(np, channels):
    """
    convert from type string to cv typedef
    """
    conversion = {
        ('uint8', 1): cv2.CV_8UC1,
        ('uint8', 2): cv2.CV_8UC2,
        ('uint8', 3): cv2.CV_8UC3,
        ('uint8', 4): cv2.CV_8UC4,

        ('int8', 1): cv2.CV_8SC1,
        ('int8', 2): cv2.CV_8SC2,
        ('int8', 3): cv2.CV_8SC3,
        ('int8', 4): cv2.CV_8SC4,

        ('uint16', 1): cv2.CV_16UC1,
        ('uint16', 2): cv2.CV_16UC2,
        ('uint16', 3): cv2.CV_16UC3,
        ('uint16', 4): cv2.CV_16UC4,

        ('int16', 1): cv2.CV_16SC1,
        ('int16', 2): cv2.CV_16SC2,
        ('int16', 3): cv2.CV_16SC3,
        ('int16', 4): cv2.CV_16SC4,

        ('int32', 1): cv2.CV_32SC1,
        ('int32', 2): cv2.CV_32SC2,
        ('int32', 3): cv2.CV_32SC3,
        ('int32', 4): cv2.CV_32SC4,

        ('float32', 1): cv2.CV_32FC1,
        ('float32', 2): cv2.CV_32FC2,
        ('float32', 3): cv2.CV_32FC3,
        ('float32', 4): cv2.CV_32FC4,

        ('float64', 1): cv2.CV_64FC1,
        ('float64', 2): cv2.CV_64FC2,
        ('float64', 3): cv2.CV_64FC3,
        ('float64', 4): cv2.CV_64FC4
    }
    return conversion[(np, channels)]
