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


#ifndef SNARK_SENSORS_DC1394_TYPES_H_
#define SNARK_SENSORS_DC1394_TYPES_H_

#include <dc1394/dc1394.h>
#include <string>

namespace snark { namespace camera {

int dc1394color_coding_to_cv_type(dc1394color_coding_t dc);

std::string video_mode_to_string( dc1394video_mode_t mode );
std::string operation_mode_to_string( dc1394operation_mode_t mode );
std::string iso_speed_to_string( dc1394speed_t speed );
std::string frame_rate_to_string( dc1394framerate_t frame_rate );
std::string color_coding_to_string( dc1394color_coding_t coding );

dc1394video_mode_t video_mode_from_string( const std::string& mode );
dc1394operation_mode_t operation_mode_from_string( const std::string& mode );
dc1394speed_t iso_speed_from_string( const std::string& speed );
dc1394framerate_t frame_rate_from_string( const std::string& frame_rate );
dc1394color_coding_t color_coding_from_string( const std::string& coding );

void print_video_modes();
void print_operation_modes();
void print_iso_speeds();
void print_frame_rates();
void print_color_coding();

} } // namespace snark { namespace camera {


#endif // SNARK_SENSORS_DC1394_TYPES_H_
