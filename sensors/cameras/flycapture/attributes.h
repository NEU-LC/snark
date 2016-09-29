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


#ifndef SNARK_SENSORS_FLYCAPTURE_ATTRIBUTES_H_
#define SNARK_SENSORS_FLYCAPTURE_ATTRIBUTES_H_

#include <string>
#include "FlyCapture2.h"
#include "flycapture.h"
#include <boost/bimap.hpp>

namespace snark { namespace cameras { namespace flycapture {
    typedef boost::bimap<FlyCapture2::PixelFormat , std::string> pixel_format_map_t;
    typedef boost::bimap<FlyCapture2::PropertyType ,std::string > property_map_t;

    std::string get_attribute( FlyCapture2::CameraBase* handle, const std::string& key );
    void set_attribute( FlyCapture2::CameraBase* handle, const std::string& key, const std::string& value );
    void set_attribute( FlyCapture2::Camera* handle, const std::string& key, const std::string& value );
    void set_attribute( FlyCapture2::GigECamera* handle, const std::string& key, const std::string& value );
    camera::attributes_type get_attributes( FlyCapture2::CameraBase* handle );
    unsigned int bits_per_pixel( const FlyCapture2::PixelFormat pixel_format_ );
    const pixel_format_map_t* get_pixel_format_map();

} } } // namespace snark { namespace cameras { namespace flycapture {

#endif // SNARK_SENSORS_FLYCAPTURE_ATTRIBUTES_H_
