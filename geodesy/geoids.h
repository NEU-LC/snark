
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

#ifndef SNARK_GEODESY_GEOIDS_H_
#define SNARK_GEODESY_GEOIDS_H_

#include "wgs84/geoid.h"
#include "agd84/geoid.h"
#include "grs67/geoid.h"

namespace snark
{
namespace geodesy
{

static const double unit_minor_semiaxis = wgs84::minor_semiaxis / wgs84::major_semiaxis;
static const double earth_average_radius = 6371000;

struct geoids
{
    //see http://en.wikipedia.org/wiki/Earth_ellipsoid
    //enum geoid_enum {wgs84};
    wgs84::geoid wgs84;
    agd84::geoid agd84;
    snark::spherical::ellipsoid sphere;
    snark::spherical::ellipsoid unit;
    grs67::geoid grs67;

    geoids():
        sphere( earth_average_radius, earth_average_radius ),
        unit( 1, unit_minor_semiaxis )
    {
    }
    void ( *info )();
    std::string name;
    snark::spherical::ellipsoid *ellipsoid;
    snark::spherical::ellipsoid *select( boost::optional< std::string > geoid_option )
    {
        if ( geoid_option )
        {
            name = *geoid_option;
            boost::to_lower( name );
            if ( name == wgs84::name )
            {
                ellipsoid = &wgs84;
                info = &wgs84::info;
            }
            else if ( name == agd84::name )
            {
                ellipsoid = &agd84;
                info = &agd84::info;
            }
            else if ( name == grs67::name )
            {
                ellipsoid = &grs67;
                info = &grs67::info;
            }
            else if ( name == "sphere" )
            {
                ellipsoid = &sphere;
                info = &sphere_info;
            }
            else if ( name == "unit" )
            {
                ellipsoid = &unit;
                info = &unit_info;
            }
            else
            {
                ellipsoid = NULL;
                info = NULL;
            }
        }
        else
        {
            name = "(default)"; //name is used for debugging only
            ellipsoid = &wgs84;
            info = &wgs84::info;
        }
        return ellipsoid;
    }
    static void other_help()
    {
        std::cerr << "        sphere: sphere with earth average radius for comparision/testing ; " << earth_average_radius << ";" << earth_average_radius << "); 1" << std::endl;
        std::cerr << "        unit: WGS84 scaled down to unit, useful for getting distance in equivalent radian for instance (1; " << unit_minor_semiaxis << ");" << wgs84::eccentricity << std::endl;
    }
    static void sphere_info()
    {
        std::cout << "shere,sphere with equal major and minor semiaxis," << earth_average_radius << "," << earth_average_radius << ",1" << std::endl;
    }
    static void unit_info()
    {
        std::cout << "unit,scaled down WGS84,1," << unit_minor_semiaxis << "," << wgs84::eccentricity << std::endl;
    }
};

}
} // namespace snark { namespace geodesy {

#endif // SNARK_GEODESY_WGS84_H_
