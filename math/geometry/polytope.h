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
// 3. All advertising materials mentioning features or use of this software
//    must display the following acknowledgement:
//    This product includes software developed by the The University of Sydney.
// 4. Neither the name of the The University of Sydney nor the
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


#ifndef SNARK_GEOMETRY_POLYGON_
#define SNARK_GEOMETRY_POLYGON_

#include <Eigen/Core>
#include <vector>
#include <comma/base/exception.h>

namespace snark{ namespace geometry{

/// convex_polytope is used to check if a point is inside a convex polytope
/// The constructor is given a convex polytope specified by a set of half-spaces. Several constructor methods exist.
/// The class works for any dimension (as long as it makes sense and your computer can handle it).
/// See test for a usage example.

class convex_polytope
{
public:
    convex_polytope( const std::vector< Eigen::VectorXd >& normals_, const std::vector< double >& offsets_ )
    {
        if(!normals_.size() || normals_[0].size())
        {
            COMMA_THROW( comma::exception, "normals size cannot be zero" );
        }
        if(normals_.size()!=offsets_.size())
        {
            COMMA_THROW( comma::exception, "normals and offsets should be of same size, got "<< normals_.size()<<" and "<<offsets_.size() );
        }
        normals.resize(normals_.size(),normals_[0].size());
        offsets.resize(offsets_.size());
        for(unsigned int cntr=0; cntr<normals_.size(); )
        {
            normals.row(cntr)=normals_[cntr].transpose();
            offsets(cntr)=offsets_[cntr];
        }
    }
    convex_polytope( const Eigen::MatrixXd& normals_, const Eigen::VectorXd& offsets_ )
    {
        if(!normals_.size())
        {
            COMMA_THROW( comma::exception, "normals size cannot be zero" );
        }
        if(normals_.rows()!=offsets_.rows())
        {
            COMMA_THROW( comma::exception, "normals and offsets should be of same size, got "<< normals_.rows()<<" and "<<offsets_.rows() );
        }
        normals=normals_;
        offsets=offsets_;
    }
    convex_polytope( const Eigen::MatrixXd& planes )
    {
        if(!planes.rows() || planes.cols()<2)
        {
            COMMA_THROW( comma::exception, "planes rows cannot be zero and cols must be at least two, got "<<planes.rows()<<" and "<<planes.cols() );
        }
        normals=planes.leftCols(planes.cols()-1);
        offsets=planes.rightCols(1);
    }

    bool has( const Eigen::VectorXd& x, double tolerance = 1e-4 );
private:
    //polytope defined by the set of inequalities: Ax>=b
    Eigen::MatrixXd normals; //A
    Eigen::VectorXd offsets; //b
};

}} // namespace snark{ namepsace geometry{

#endif // #ifndef SNARK_GEOMETRY_POLYGON_
