// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2014 The University of Sydney
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

#include <Eigen/Eigen>
#include <comma/base/exception.h>
#include <comma/math/compare.h>
#include "n_sphere.h"

namespace snark { namespace geometry {

n_sphere::n_sphere( const Eigen::VectorXd& center, const double radius )
    : center( center )
    , radius( radius )
    , squared_radius( radius * radius )
{
}

bool n_sphere::contains( const Eigen::VectorXd& p, const double epsilon ) const
{
    if( p.size() != center.size() ) { COMMA_THROW( comma::exception, "expected same dimension as n-sphere: "<< center.size() << "; got dimension: " << p.size() ); }
    return !comma::math::less( squared_radius, ( p - center ).squaredNorm(), epsilon );
}

bool n_sphere::intersects( const n_sphere& s, const double epsilon ) const
{
    if( s.center.size() != center.size() ) { COMMA_THROW( comma::exception, "expected same dimension as n-sphere: "<< center.size() << "; got dimension: " << s.center.size() ); }
    return !comma::math::less( s.radius + radius, ( s.center - center ).norm(), epsilon );
}

} } // namespace snark{ { namespace geometry {
