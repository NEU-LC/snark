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

#ifndef SNARK_GEODESY_WGS84_H_
#define SNARK_GEODESY_WGS84_H_

#include "../../math/spherical_geometry/ellipsoid.h"

namespace snark
{
namespace geodesy
{
namespace wgs84
{

static const double eccentricity = 1.0 / 298.257223563;
static const double major_semiaxis = 6378137.0;
static const double minor_semiaxis = 6356752.314245;
static const double radius = major_semiaxis;
static const std::string name( "wgs84" );

void inline help()
{
    std::cerr << "        WGS84: World Geodesic System 1984 standard used in GPS (6378137.0; 6356752.314245); 298.257223563" << std::endl;
}

void inline info()
{
    std::cout << "WGS84,World Geodesic System 1984," << major_semiaxis << "," << minor_semiaxis << "," << eccentricity << std::endl;
}

struct geoid : public spherical::ellipsoid
{
    geoid() : ellipsoid( wgs84::major_semiaxis, wgs84::minor_semiaxis ) {}
};

}
}
} // namespace snark { namespace geodesy { namespace wgs84 {

#endif // SNARK_GEODESY_WGS84_H_
