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

#ifndef SNARK_GEODESY_AGD84_H_
#define SNARK_GEODESY_AGD84_H_

#include "../../math/spherical_geometry/ellipsoid.h"
#include <sstream>

namespace snark
{
namespace geodesy
{
namespace agd84
{

static const double eccentricity = 1.0 / 298.25;
static const double major_semiaxis = 6378160.0;
static const double minor_semiaxis = 6356774.719;
static const double radius = major_semiaxis;
static const std::string name( "agd84" );

/* cahced from  http://georepository.com/datum_6203/Australian-Geodetic-Datum-1984.html

Geodetic Datum used in Australia - AGD84

Australian Geodetic Datum 1984 is a geodetic datum first defined in 1984 and is suitable for use in Australia - Queensland (Qld), South Australia (SA), Western Australia (WA).. Australian Geodetic Datum 1984 references the Australian National Spheroid ellipsoid and the Greenwich prime meridian. Australian Geodetic Datum 1984 origin is Fundamental point: Johnson Memorial Cairn. Latitude: 25°56'54.5515"S, longitude: 133°12'30.0771"E (of Greenwich). Australian Geodetic Datum 1984 is a geodetic datum for Topographic mapping. It was defined by information from "GDA technical manual v2_2", Intergovernmental Committee on Surveying and Mapping. www.anzlic.org.au/icsm/gdtm/ Uses all data from 1966 adjustment with additional observations, improved software and a geoid model.

Datum Details
DATUM NAME:     Australian Geodetic Datum 1984
CODE:   6203
AREA OF USE:    Australia - AGD84Open

*********************************
Ellipsoid Details
NAME:   Australian National Spheroid
CODE:   7003
SHAPE:  Ellipsoid
SEMI-MAJOR AXIS:        6378160 metreOpen
INVERSE FLATTENING:     298.25

META DATA
REMARKS:
Based on the GRS 1967 figure but with 1/f taken to 2 decimal places exactly. The dimensions are also used as the GRS 1967 Modified ellipsoid (see code 7050).
INFORMATION SOURCE:     "Australian Map Grid Technical Manual"; National Mapping Council of Australia Special Publication #7; 1972
DATA SOURCE:    OGP
REVISION DATE:  08/11/2008
CHANGE ID:      [2002.500] [2008.017]

*/

inline std::string help()
{
    std::stringstream os;
    os << "        AGD84: Australian Geodetic Datum 1984 - uses Australian National 1966 spheroid (6,378,160; 6,356,774.719); 298.25" << std::endl;
    return os.str();
}

inline std::string info()
{
    std::stringstream os;
    os << "AGD84,Australian Geodetic Datum 1984," << major_semiaxis << "," << minor_semiaxis << "," << eccentricity << std::endl;
    return os.str();
}

struct geoid : public spherical::ellipsoid
{
    geoid() : ellipsoid( agd84::major_semiaxis, agd84::minor_semiaxis ) {}
};

}
}
} // namespace snark { namespace geodesy { namespace agd84 {

#endif // SNARK_GEODESY_AGD84_H_
