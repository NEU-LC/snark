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

/// @authors vsevolod vlaskine, zhe xu

#ifndef SNARK_IMAGING_CAMERA_CONFIG_H
#define SNARK_IMAGING_CAMERA_CONFIG_H

#include <boost/optional.hpp>
#include <Eigen/Core>

namespace snark { namespace camera {

struct config
{
    struct distortion_t
    {
        struct radial_t
        {
            double k1, k2, k3;
            
            radial_t() : k1( 0 ), k2( 0 ), k3( 0 ) {}
        };
        
        struct tangential_t
        {
            double p1, p2;
            
            tangential_t() : p1( 0 ), p2( 0 ) {}
        };
        
        radial_t radial;
        tangential_t tangential;
        
        operator Eigen::Matrix< double, 5, 1 >() const;
    };

    /// sensor size in metres
    Eigen::Vector2d sensor_size;
    
    /// image size in pixels
    Eigen::Vector2d image_size;
    
    /// focal length in pixels
    Eigen::Vector2d focal_length;
    
    /// principal point in pixels; if not defined, then image centre
    boost::optional< Eigen::Vector2d > principal_point;
    
    /// distortion
    distortion_t distortion;
    
    config() : sensor_size( Eigen::Vector2d::Zero() ), image_size( Eigen::Vector2d::Zero() ), focal_length( Eigen::Vector2d::Zero() ), principal_point( Eigen::Vector2d::Zero() ) {}
};
    
} } // namespace snark { namespace camera {

#endif // SNARK_IMAGING_CAMERA_CONFIG_H
