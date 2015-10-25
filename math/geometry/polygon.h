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

/// @author vsevolod vlaskine

#ifndef SNARK_MATH_GEOMETRY_POLYGON_
#define SNARK_MATH_GEOMETRY_POLYGON_

#include <Eigen/Core>
#include <vector>
#include <boost/array.hpp>

namespace snark {

/// planar convex polygon
struct convex_polygon
{
    /// corners
    std::vector< Eigen::Vector3d > corners;
    
    /// default constructor
    convex_polygon() {}
    
    /// constructor
    convex_polygon( const std::vector< Eigen::Vector3d >& corners ) : corners( corners ) {}
    
    /// normal to polygon plane
    Eigen::Vector3d normal() const;
    
    /// return true, if planar and convex
    bool is_valid() const;
    
    /// return projection of a point on the polygon plane
    Eigen::Vector3d projection_of( const Eigen::Vector3d& rhs ) const;
    
    /// return true, if a point is inside of the polygon, borders included
    bool includes( const Eigen::Vector3d& rhs ) const;
};

/// triangle, a convenience class, since it's so commonly used
struct triangle
{
    /// corners
    boost::array< Eigen::Vector3d, 3 > corners;
    
    /// default constructor
    triangle() {}
    
    /// constructor
    triangle( const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c ) { corners[0] = a; corners[1] = b; corners[2] = c; }
    
    /// normal to triangle plane
    Eigen::Vector3d normal() const;
    
    /// return projection of a point on the triangle plane
    Eigen::Vector3d projection_of( const Eigen::Vector3d& rhs ) const;
    
    /// return true, if a point is inside of the triangle, borders included
    bool includes( const Eigen::Vector3d& rhs ) const;
};

} // namespace snark {
    
#endif // #ifndef SNARK_MATH_GEOMETRY_POLYGON_
