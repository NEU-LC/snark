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

#include <cmath>
#include <boost/array.hpp>
#include "calculator.h"
#include "packet.h"

namespace snark { namespace velodyne { namespace puck {

static boost::array< double, 16 > elevation_ = {{ -15.0 * M_PI / 180
                                                , 1 * M_PI / 180
                                                , -13 * M_PI / 180
                                                , 3 * M_PI / 180
                                                , -11 * M_PI / 180
                                                , 5 * M_PI / 180
                                                , -9 * M_PI / 180
                                                , 7 * M_PI / 180
                                                , -7 * M_PI / 180
                                                , 9 * M_PI / 180
                                                , -5 * M_PI / 180
                                                , 11 * M_PI / 180
                                                , -3 * M_PI / 180
                                                , 13 * M_PI / 180
                                                , -1 * M_PI / 180
                                                , 15 * M_PI / 180 }};

struct laser
{
    double sin;
    double cos;
    
    laser() {}
    laser( unsigned int index ) : sin( std::sin( elevation_[ index ] ) ), cos( std::cos( elevation_[ index ] ) ) {}
};

typedef boost::array< laser, puck::packet::number_of_lasers > lasers_t;

static lasers_t init_lasers()
{
    lasers_t lasers;
    for( unsigned int j = 0; j < puck::packet::number_of_lasers; ++j ) { lasers[j] = laser( j ); }
    return lasers;
}

static lasers_t lasers = init_lasers();

std::pair< ::Eigen::Vector3d, ::Eigen::Vector3d > calculator::ray( unsigned int laser, double range, double angle ) const { return std::make_pair( ::Eigen::Vector3d::Zero(), point( laser, range, angle ) ); }

::Eigen::Vector3d calculator::point( unsigned int laser, double range, double angle ) const
{
    // todo: once puck/packet.cpp is fixed, use the commented line below
    // return ::Eigen::Vector3d( range * lasers[laser].cos * std::cos( angle ), range * lasers[laser].cos * std::sin( angle ), range * lasers[laser].sin );
    return ::Eigen::Vector3d( range * lasers[laser].cos * std::sin( angle ), range * lasers[laser].cos * std::cos( angle ), range * lasers[laser].sin );
}

double calculator::range( unsigned int, double range ) const { return range; }

// todo! super quick and dirty; by right the places to fix:
//       - puck/packet.cpp: fiddly part: fix azimuth and make sure step works correctly
//       - puck/calculator.cpp: easy part: simply swap sin and cos in calculator::point()
//       - scan_tick.cpp: packet_angle_( puck::packet ): hardcoded offset
//       - if time permits, see why there is such a 90-degree discrepancy in puck spec
// double calculator::azimuth( unsigned int, double azimuth ) const { return azimuth; }
double calculator::azimuth( unsigned int, double azimuth ) const { return M_PI / 2 - azimuth; }

double calculator::intensity( unsigned int, unsigned char intensity, double ) const { return intensity; }

} } } // namespace snark { namespace velodyne { namespace puck {
