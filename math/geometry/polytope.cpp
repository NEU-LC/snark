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


#include <cmath>
#include <Eigen/Eigen>
#include <comma/base/exception.h>
#include "../rotation_matrix.h"
#include "polytope.h"

namespace snark { namespace geometry {

convex_polytope::convex_polytope( const std::vector< Eigen::VectorXd >& normals, const std::vector< double >& distances )
    : normals_( normals.size(), normals[0].size() )
    , distances_( distances.size() )
{
    if( !normals.size() || !normals[0].size() ) { COMMA_THROW( comma::exception, "normals cannot be empty or have zero dimensions" ); }
    if( normals.size() != distances.size() ) { COMMA_THROW( comma::exception, "normals and distances should be of same size, got "<< normals.size()<<" normals and " << distances.size() << " distances" ); }
    for( unsigned int i = 0; i < normals.size(); ++i )
    {
        normals_.row( i ) = normals[i].transpose().normalized();
        distances_( i ) = distances[i];
    }
}

convex_polytope::convex_polytope( const Eigen::MatrixXd& normals, const Eigen::VectorXd& distances )
    : normals_( normals )
    , distances_( distances )
{
    if( !normals.size() ) { COMMA_THROW( comma::exception, "normals cannot be empty" ); }
    if( normals.rows() != distances.size() ) { COMMA_THROW( comma::exception, "normals and distances should be of same size, got "<< normals.rows() << " normals and " << distances.size() << " distances" ); }
    for( unsigned int i = 0; i < normals.rows(); ++i ) { normals_.row( i ).normalize(); }
}

convex_polytope convex_polytope::transformed( const Eigen::Vector3d& translation, const snark::roll_pitch_yaw& orientation ) const
{
    convex_polytope p( *this );
    const Eigen::MatrixXd& rotation_matrix = snark::rotation_matrix::rotation( orientation );
    for( unsigned int i = 0; i < normals_.rows(); ++i ) // todo: quick and dirty; can it be done in a single operation?
    {
        p.normals_.row( i ) = rotation_matrix * normals_.row( i ).transpose();
        const Eigen::VectorXd& n = p.normals_.row( i );
        p.distances_[i] = n.dot( n * distances_[i] + translation );
    }
    return p;
}

bool convex_polytope::has( const Eigen::VectorXd &rhs, double epsilon ) const
{
    if( rhs.size() != normals_.cols() ) { COMMA_THROW( comma::exception, "expected same dimension as polytope: "<< normals_.cols()<<"; got dimension: " << rhs.size() ); }
    return ( ( normals_ * rhs - distances_ ).array() <= epsilon ).all();
}

} } // namespace snark{ { namespace geometry {
