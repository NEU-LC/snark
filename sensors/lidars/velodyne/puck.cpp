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
#include "puck.h"

namespace snark { namespace velodyne {

static boost::array< double, 16 > elevation_ = {{ -15.0 * 180 / M_PI
                                                , 1 * 180 / M_PI
                                                , -13 * 180 / M_PI
                                                , -3 * 180 / M_PI
                                                , -11 * 180 / M_PI
                                                , 5 * 180 / M_PI
                                                , -9 * 180 / M_PI
                                                , 7 * 180 / M_PI
                                                , -7 * 180 / M_PI
                                                , 9 * 180 / M_PI
                                                , -5 * 180 / M_PI
                                                , 11 * 180 / M_PI
                                                , -3 * 180 / M_PI
                                                , 13 * 180 / M_PI
                                                , -1 * 180 / M_PI
                                                , 15 * 180 / M_PI }};

boost::array< double, 16 > get_sin_( const boost::array< double, 16 >& e )
{
    boost::array< double, 16 > s;
    for( unsigned int i = 0; i < s.size(); ++i ) { s[i] = std::sin( e[i] ); }
    return s;
}

boost::array< double, 16 > get_cos_( const boost::array< double, 16 >& e )
{
    boost::array< double, 16 > s;
    for( unsigned int i = 0; i < s.size(); ++i ) { s[i] = std::cos( e[i] ); }
    return s;
}

static boost::array< double, 16 > elevation_sin_ = get_sin_( elevation_ );

static boost::array< double, 16 > elevation_cos_ = get_cos_( elevation_ );

std::pair< ::Eigen::Vector3d, ::Eigen::Vector3d > puck::ray( unsigned int laser, double range, double angle ) const { return std::make_pair( ::Eigen::Vector3d::Zero(), point( laser, range, angle ) ); }

::Eigen::Vector3d puck::point( unsigned int laser, double range, double angle ) const
{
    laser = laser % 16; // todo: fix it!
    return ::Eigen::Vector3d( range * elevation_cos_[laser] * std::sin( angle ), range * elevation_cos_[laser] * std::cos( angle ), range * elevation_sin_[laser] );
}

double puck::range( unsigned int, double range ) const { return range; }

double puck::azimuth( unsigned int, double azimuth ) const { return azimuth; }

double puck::intensity( unsigned int, unsigned char intensity, double ) const { return intensity; }

} } // namespace snark { namespace velodyne {
