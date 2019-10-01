// This file is provided in addition to snark and is not an integral
// part of snark library.
// Copyright (c) 2019 Vsevolod Vlaskine
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
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

// snark is a generic and flexible library for robotics research
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

/// @author vsevolod vlaskine

#pragma once

#include <Eigen/Core>
#include "../math/range_bearing_elevation.h"

namespace snark { namespace equirectangular {

namespace cube {

/// cube face enumeration
struct faces { enum values { top = 0, back, left, front, right, bottom }; };

/// @param p normalized pixel coordinates on cube face
/// @param face cube face
/// @returns coordinates on unit cube with centre at 0,0,0
Eigen::Vector3d to_cartesian( const Eigen::Vector2d p, cube::faces::values face );
    
} // namespace cube {

/// @param v cartesian coordinates of pixel
/// @returns cube face
cube::faces::values face_of( const Eigen::Vector3d& v );

/// @param p pixel coordinate in spherical image
/// @param spherical_width width of spherical image in pixels
/// @returns normalized pixel on spherical image of size 1x0.5
Eigen::Vector2d normalized( const Eigen::Vector2d& p, double spherical_width );

/// @param p pixel coordinate in spherical image
/// @param spherical_width width of spherical image in pixels
/// @returns coordinates on cube face normalized to [0,1) and cube face id
std::pair< Eigen::Vector2d, cube::faces::values > to_cube( const Eigen::Vector2d& p, double spherical_width );

/// @param p normalized pixel coordinate in spherical image
/// @returns coordinates on cube face normalized to [0,1) and cube face id
std::pair< Eigen::Vector2d, cube::faces::values > to_cube( const Eigen::Vector2d& p );

/// @param p pixel coordinate in spherical image
/// @param spherical_width width of spherical image in pixels
/// @returns range-bearing-elevation for point on sphere inscribed into a unit cube (i.e. sphere radius is 0.5)
snark::range_bearing_elevation to_polar( const Eigen::Vector2d& p, double spherical_width );

/// @param p normalized pixel coordinate in spherical image
/// @returns range-bearing-elevation for point on sphere inscribed into a unit cube (i.e. sphere radius is 0.5)
snark::range_bearing_elevation to_polar( const Eigen::Vector2d& p );

/// @param p normalized pixel coordinate in spherical image
/// @returns coordinates on unit cube with centre at 0,0,0
Eigen::Vector3d to_cartesian( const Eigen::Vector2d& p );

/// @param p normalized pixel coordinate on a cube face
/// @param face cube face
/// @returns coordinates on spherical images normalized to a spherical of size 1x0.5
Eigen::Vector2d from_cube( const Eigen::Vector2d& p, cube::faces::values face );
    
} } // namespace snark { namespace equirectangular {
