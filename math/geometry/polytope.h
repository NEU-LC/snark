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
#include "../roll_pitch_yaw.h"

namespace snark { namespace geometry {

/// convex_polytope is used to check if a point is inside a convex polytope specified by a set of half-spaces,
/// i.e. by the set of inequalities: Ax <= b, where A is plane normals and b plane offsets (signed
/// distances from zero to planes), assuming that normals point outwards of the polytope

class convex_polytope
{
    public:
        /// @param normals to the planes
        /// @param offsets from the origins to the planes
        convex_polytope( const std::vector< Eigen::VectorXd >& normals, const std::vector< double >& distances );

        /// @param normals to the planes
        /// @param offsets from the origins to the planes
        convex_polytope( const Eigen::MatrixXd& normals, const Eigen::VectorXd& offsets );

        /// @return true, if point is inside of the polytope
        bool has( const Eigen::VectorXd& rhs, double epsilon = 0 ) const;

        /// @return rotated and translated polytope
        convex_polytope transformed( const Eigen::Vector3d& translation, const snark::roll_pitch_yaw& rotation );

        const Eigen::MatrixXd& normals() const { return normals_; }

        const Eigen::VectorXd& distances() const { return distances_; }

    private:
        Eigen::MatrixXd normals_;
        Eigen::VectorXd distances_;
};

} } // namespace snark { namepsace geometry {

#endif // SNARK_MATH_GEOMETRY_POLYTOPE_
