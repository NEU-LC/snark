#!/usr/bin/env python

import numpy as np
import skimage
from skimage import color
import random

random.seed( 0 );

def rgb2ycbcr():
    name="rgb2ycbcr"
    inputs = [ ( 255, 0, 0 ),
               ( 0, 255, 0 ),
               ( 0, 0, 255 ),
               ( 12, 56, 43 ) ]
    for i in xrange( 124 ):
        inputs.append( ( random.randint( 0, 255 ), random.randint( 0, 255 ), random.randint( 0, 255 ) ) )
    with open("input", "wt") as fi, open("expected", "wt" ) as fe:
        counter = 0
        for i in inputs:
            pixel_rgb = np.array( [[i]], dtype=np.uint8 )
            pixel_ycbcr = color.rgb2ycbcr( pixel_rgb )
            t = (name, counter,) + i
            fi.write( '%s[%d]="echo %d,%d,%d | image-color-calc convert --from rgb,ub --to ycbcr,ub"\n' % t )
            t = (name, counter,) + i + tuple( pixel_ycbcr[0][0].round().tolist() ) + (name,counter,)
            fe.write( '%s[%d]/output="%d,%d,%d,%d,%d,%d"\n%s[%d]/status=0\n' % t )
            counter = counter + 1

if __name__ == '__main__':
    rgb2ycbcr()
