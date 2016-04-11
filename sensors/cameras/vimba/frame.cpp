// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2016 The University of Sydney
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

#include <iostream>
#include <comma/base/exception.h>
#include "error.h"
#include "frame.h"

namespace snark { namespace vimba {

frame::frame( const AVT::VmbAPI::FramePtr& frame_ptr )
    : image_buffer_( NULL )
    , last_frame_id_( 0 )
{
    VmbErrorType status;
    status = SP_ACCESS( frame_ptr )->GetFrameID( frame_id_ );
    if( status != VmbErrorSuccess ) {
        COMMA_THROW( comma::exception, error_msg( "GetFrameID() failed", status ));
    }
    status = SP_ACCESS( frame_ptr )->GetReceiveStatus( frame_status_ );
    if( status != VmbErrorSuccess ) {
        COMMA_THROW( comma::exception, error_msg( "GetReceiveStatus() failed", status ));
    }
    status = SP_ACCESS( frame_ptr )->GetHeight( height_ );
    if( status != VmbErrorSuccess ) {
        COMMA_THROW( comma::exception, error_msg( "GetHeight() failed", status ));
    }
    status = SP_ACCESS( frame_ptr )->GetWidth( width_ );
    if( status != VmbErrorSuccess ) {
        COMMA_THROW( comma::exception, error_msg( "GetWidth() failed", status ));
    }
    status = SP_ACCESS( frame_ptr )->GetImageSize( size_ );
    if( status != VmbErrorSuccess ) {
        COMMA_THROW( comma::exception, error_msg( "GetImageSize() failed", status ));
    }
    status = SP_ACCESS( frame_ptr )->GetImage( image_buffer_ );
    if( status != VmbErrorSuccess ) {
        COMMA_THROW( comma::exception, error_msg( "GetImage() failed", status ));
    }
    status = SP_ACCESS( frame_ptr )->GetPixelFormat( pixel_format_ );
    if( status != VmbErrorSuccess ) {
        COMMA_THROW( comma::exception, error_msg( "GetPixelFormat() failed", status ));
    }
}

void frame::check_id()
{
    if( last_frame_id_ != 0 )
    {
        VmbUint64_t missing_frames = frame_id_ - last_frame_id_ - 1;
        if( missing_frames > 0 )
        {
            std::cerr << "Warning: " << missing_frames << " missing frame"
                      << ( missing_frames == 1 ? "" : "s" )
                      << " detected" << std::endl;
        }
    }
    last_frame_id_ = frame_id_;
}

void frame::check_status() const
{
    if( frame_status_ != VmbFrameStatusComplete )
    {
        std::cerr << "Warning: frame " << frame_id_ << " status "
                  << frame_status_string()
                  << " (" << width_ << "x" << height_ << " with " << size_ << " bytes)" << std::endl;
    }
}

std::string frame::frame_status_string() const
{
    switch( frame_status_ )
    {
        case VmbFrameStatusComplete:   return "Complete";
        case VmbFrameStatusIncomplete: return "Incomplete";
        case VmbFrameStatusTooSmall:   return "Too small";
        case VmbFrameStatusInvalid:    return "Invalid";
    }
    return "";                          // Quiet gcc warning
}

} } // namespace snark { namespace vimba {
