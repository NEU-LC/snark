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

#ifndef SNARK_MATH_SPHERICAL_GEOMETRY_ELLIPSOID_H_
#define SNARK_MATH_SPHERICAL_GEOMETRY_ELLIPSOID_H_

#include <vector>
#include <boost/optional.hpp>
#include "coordinates.h"

namespace snark
{
namespace spherical
{

struct ellipsoid
{
    /// constructor
    ellipsoid( double major_semiaxis, double minor_semiaxis );

    double major_semiaxis;

    double minor_semiaxis;

    /// return distance between from and to coordinates, using vincenty's formulae
    double distance( const coordinates &from, const coordinates &to );

    /// return coordinates at a given distance from begin at given bearing, using vincenty's formulae
    /// todo: an extensive comment no what kind of bearing is used
    coordinates at( const coordinates &begin, double distance, double bearing );

    //heading degree from north; east is positive 90
    static double bearing( double heading )
    {
        //bearing standard geometric radian; ie east=0,north=pi/2,south=-pi/2
        return ( heading * -1 ) + ( M_PI / 2 );
    }

    //great circle arc
    struct arc
    {
        coordinates begin;
        coordinates end;
    };

    //small circle
    struct circle
    {
        //spherical coordinates in radian
        coordinates centre;
        //radius is distance in metric unit
        double radius;
        struct arc;
        std::vector< coordinates > discretize( ellipsoid &ellipsoid, const boost::optional< double > &resolution, const boost::optional< unsigned int > &circle_size ) const;
    };

    //circular arc
    struct circle::arc
    {
        /// circle of which this is an arc
        ellipsoid::circle circle;
        /// begin bearing
        double begin;
        /// end bearing
        double end;
        arc() {}
        arc( const ellipsoid::circle &circle, double begin, double end ) : circle( circle ), begin( begin ), end( end ) {}
        std::vector< coordinates > discretize( ellipsoid &ellipsoid, const boost::optional< double > &resolution, const boost::optional< unsigned int > &circle_size ) const;
    };

};

}
} // namespace snark { namespace spherical {

#endif // SNARK_MATH_SPHERICAL_GEOMETRY_ELLIPSOID_H_
