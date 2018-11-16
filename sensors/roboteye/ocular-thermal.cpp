// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2018 The University of Sydney
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

#include "ocular-thermal.h"
#include <RobotEye.h>
#include <comma/base/exception.h>
#include <comma/application/verbose.h>
#include <opencv2/core/types_c.h>

namespace snark { namespace ocular { namespace roboteye { 

::ocular::ocular_error_t check_status( ::ocular::ocular_error_t status, const std::string& msg )
{
    if( status != ::ocular::NO_ERR )
    {
        std::cerr << comma::verbose.app_name() << ": " << msg << ( msg.empty() ? "" : ": " )
                  << ::ocular::RobotEye::GetErrorString( status ) << std::endl;
    }
    return status;
}

::ocular::dev_status_t check_dev_status( ::ocular::dev_status_t status, const std::string& msg )
{
    if( status.devCode != ::ocular::NO_ERR )
    {
        std::cerr << comma::verbose.app_name() << ": " << msg << ( msg.empty() ? "" : ": " )
                  << ::ocular::RobotEyeThermal::GetErrorString( status ) << std::endl;
    }
    return status;
}

int pixel_type_to_opencv( uint8_t pixel_type )
{
    switch( pixel_type )
    {
        case ::ocular::thermal::PIXELTYPE_MONO_8:  return CV_8UC1;
        case ::ocular::thermal::PIXELTYPE_MONO_14: return CV_16UC1;
        default: COMMA_THROW( comma::exception, "unsupported pixel type " << pixel_type );
    }
}
    
} } } //namespace snark { namespace ocular { namespace roboteye { 
