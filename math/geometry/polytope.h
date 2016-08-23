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

/// @author abdallah kassir

#ifndef SNARK_MATH_GEOMETRY_POLYTOPE_
#define SNARK_MATH_GEOMETRY_POLYTOPE_

#include <vector>
#include <Eigen/Core>
#include <comma/base/exception.h>

namespace snark { namespace geometry {

/// convex_polytope is used to check if a point is inside a convex polytope
/// The constructor is given a convex polytope specified by a set of half-spaces. Several constructor methods exist.
/// The class works for any dimension (as long as it makes sense and your computer can handle it).
/// See test for a usage example.

class convex_polytope
{
public:
    /// @param normals to the planes
    /// @param offsets from the origins to the planes
    convex_polytope( const std::vector< Eigen::VectorXd >& normals, const std::vector< double >& offsets );
    
    /// @param normals to the planes
    /// @param offsets from the origins to the planes
    convex_polytope( const Eigen::MatrixXd& normals, const Eigen::VectorXd& offsets );
    
    /// @param planes normals and offsets concatenated into one matrix
    convex_polytope( const Eigen::MatrixXd& planes );
    
    /// construct from vertices and faces
    /// a convenience constructor, since it is highly redundant
    /// no convexity checks are performed
    /// if a face has more than 3 vertices, no planarity checks are performed
    template < template < typename > class Container >
    convex_polytope( const std::vector< Eigen::Vector3d >& vertices, const Container< Eigen::Vector3d >& faces );

    /// @return true, if point is inside of the polytope
    bool has( const Eigen::VectorXd& x ) const;
    
    /// return normals
    const Eigen::MatrixXd& normals() const;
    
    /// return offsets
    const Eigen::VectorXd& offsets() const;
    
private:
    //polytope defined by the set of inequalities: Ax>=b
    Eigen::MatrixXd normals_; //A
    Eigen::VectorXd offsets_; //b
};

template < template < typename > class Container >
inline convex_polytope::convex_polytope( const std::vector< Eigen::Vector3d >& vertices, const Container< Eigen::Vector3d >& faces )
    : normals_( faces.size(), vertices.begin()->size() )
    , offsets_( faces.size() )
{
    unsigned int k = 0;
    for( typename Container< Eigen::Vector3d >::const_iterator it = faces.begin(); it != faces.end(); ++it, ++k )
    {
        std::vector< Eigen::Vector3d > face( it->size() );
        if( face.size() < 3 ) { COMMA_THROW( comma::exception, "expected face with at least 3 vertices, got: " << face.size() ); }
        for( unsigned int i = 0; i < 3; ++i ) { if( *it[i] >= vertices.size() ) { COMMA_THROW( comma::exception, "face " << k << " invalid: expected vertex index less than " << vertices.size() << ", got: " << *it[i] ); } }
        const Eigen::Vector3d& normal = ( *it[2] - *it[1] ).cross( *it[2] - *it[1] ).normalized();
        offsets_[k] = normal.dot( *it[0] ); // todo: debug
        normals_[k] = normal; // todo: debug
    }
}

} } // namespace snark { namepsace geometry {

#endif // SNARK_MATH_GEOMETRY_POLYTOPE_
