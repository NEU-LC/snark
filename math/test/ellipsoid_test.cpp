// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2011 The University of Sydney
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of the University of Sydney nor the
//    names of its contributors may be used to endorse or promote products
//    derived from this software without specific prior written permission.
//
// NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
// GRANTED BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
// HOLDERS AND CONTRIBUTORS \"AS IS\" AND ANY EXPRESS OR IMPLIED
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
// IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <gtest/gtest.h>
#include "../spherical_geometry/ellipsoid.h"

using namespace snark::spherical;

namespace snark { namespace math {

#define rad(x) (((x) * pi)/180.0)
#define pos(x,y) coordinates(rad(x),rad(y))
    
TEST(ellipsoid, distance)
{
    const double pi = std::atan(1.0) * 4;

    //in km
    ellipsoid e(6378.1370, 6356.7523);
    EXPECT_EQ(1,3-2);
    
    //border conditions
    EXPECT_NEAR(e.distance(pos(90 ,12), pos(90,34)), 0, .000001);

    //EXPECT_NEAR(e.distance(coordinates(0,0), coordinates(90,0)), 10001965.729, 0.001);
    EXPECT_NEAR(e.distance(pos(0 ,0), pos(90,0)), 10001.965729, 0.00005);
    
    EXPECT_NEAR(e.distance(pos(0 ,0), pos(0,90)), (pi*6378.137) / 2 , 0.00005);
    // 0,0 to 0,180 will fail to converge
    
    //perth-syd YPPH (PER) – Pert
    coordinates Perth(rad(-31.9402777778),rad(115.966944444));
    //YSSY (SYD) – Sydney
    coordinates Syndey(rad(-33.9461111111),rad(151.177222222));

    //EXPECT_NEAR(e.distance(Perth,Syndey), 3284061.441, 0.1);
    EXPECT_NEAR(e.distance(Perth,Syndey), 3284.061441, 0.00005);
    
    //equal measure
    {
        //hemisphere symetry
        EXPECT_DOUBLE_EQ(e.distance(pos(32,0),pos(32,45)), e.distance(pos(-32,0),pos(-32,45)));
        
        EXPECT_NEAR(e.distance(pos(18, 22), pos(23, 62)), 4196.427624, 0.000001);
        
        //lateral transform
        {
            const double a=18;
            const double b=23;
            const double d=40;
            
            EXPECT_DOUBLE_EQ(e.distance(pos(a,22),pos(b,22+d)), e.distance(pos(a,85),pos(b,85+d)));
        }
        
        //longitudinal arcs
        {
            for (int i=0;i<10;i++)
                EXPECT_DOUBLE_EQ(e.distance(pos(25,0),pos(55,0)), e.distance(pos(25,i*10),pos(55,i*10)));
        }
    }
    
    
}


}}