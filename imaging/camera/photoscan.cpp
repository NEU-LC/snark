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

#include <comma/base/exception.h>
#include "photoscan.h"

namespace snark { namespace photoscan {

template < typename T > T photoscan::camera::pinhole::calibration_t::as() const { COMMA_THROW( comma::exception, "not implemented, just ask" ); }

template <> snark::camera::pinhole::config_t photoscan::camera::pinhole::calibration_t::as< snark::camera::pinhole::config_t >() const
{
    snark::camera::pinhole::config_t config;
    config.image_size = Eigen::Vector2i( width, height );
    config.focal_length = f;
    config.principal_point = Eigen::Vector2d( width / 2. + cx, height / 2. + cy );
    config.distortion = snark::camera::pinhole::config_t::distortion_t();
    config.distortion->radial.k1 = k1;
    config.distortion->radial.k2 = k2;
    config.distortion->radial.k3 = k3;
    config.distortion->tangential.p1 = p2; // sic!
    config.distortion->tangential.p2 = p1; // sic!
    return config;
}

} } // namespace snark { namespace photoscan {
