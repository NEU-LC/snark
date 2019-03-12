// This file is part of snark, a generic and flexible library for robotics research
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

#include "../../math/frame_transforms.h"
#include "pinhole.h"

namespace snark { namespace camera { namespace stereo {

class pair
{
    public:
        typedef std::pair< camera::config, camera::config > config_t;
        
        struct camera_t
        {
            snark::camera::pinhole pinhole;
            
            snark::pose pose;
            
            camera_t( const snark::camera::config& config ): pinhole( config.pinhole ), pose( config.pose ) {}
            
            camera_t( const snark::camera::pinhole::config_t& config, const snark::pose& pose ): pinhole( config ), pose( pose ) {}
        };
        
        pair( const config_t& config );
        
        pair( const pinhole::config_t& first );
        
        pair( const pinhole::config_t& first, double baseline );
        
        pair( const pinhole::config_t& first, const pinhole::config_t& second );
        
        pair( const pinhole::config_t& first, const pinhole::config_t& second, double baseline );
        
        const camera_t& first() const { return first_; }
        
        const camera_t& second() const { return second_; }
        
        std::pair< Eigen::Vector3d, Eigen::Vector3d > to_cartesian( const Eigen::Vector2d& first, const Eigen::Vector2d& second ) const;
        
        std::pair< Eigen::Vector3d, Eigen::Vector3d > to_cartesian( const Eigen::Vector2d& first, const Eigen::Vector2d& second, const snark::pose& pose ) const;
        
        std::pair< Eigen::Vector3d, Eigen::Vector3d > to_cartesian( const Eigen::Vector2d& first, const Eigen::Vector2d& second, const snark::pose& first_pose, const snark::pose& second_pose ) const;
        
    private:
        camera_t first_;
        camera_t second_;
};

} } } // namespace snark { namespace camera { namespace stereo {
