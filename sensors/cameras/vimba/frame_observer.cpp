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
#include <comma/application/verbose.h>
#include "snark/imaging/cv_mat/serialization.h"
#include "error.h"
#include "frame_observer.h"

namespace snark { namespace vimba {

frame_observer::frame_observer( AVT::VmbAPI::CameraPtr camera
                              , std::unique_ptr< snark::cv_mat::serialization > serialization )
    : IFrameObserver( camera )
    , last_frame_id_( 0 )
    , serialization_( std::move( serialization ))
{
    std::cerr << "Creating frame_observer" << std::endl;
}

void frame_observer::FrameReceived( const AVT::VmbAPI::FramePtr frame )
{
    VmbUint32_t height;
    SP_ACCESS( frame )->GetHeight( height );

    VmbUint32_t width;
    SP_ACCESS( frame )->GetWidth( width );

    VmbUchar_t* image_buffer;
    SP_ACCESS( frame )->GetImage( image_buffer );

    check_frame_id( frame );
    check_frame_status( frame );

    // TODO: different image formats
    cv::Mat cv_mat( height, width * 1.5, CV_8UC1, image_buffer );

    serialization_->write( std::cout
                         , std::make_pair( boost::posix_time::microsec_clock::universal_time(), cv_mat ));

    m_pCamera->QueueFrame( frame );
}

std::string frame_observer::frame_status_to_string( VmbFrameStatusType frame_status )
{
    switch( frame_status )
    {
        case VmbFrameStatusComplete:   return "Complete";
        case VmbFrameStatusIncomplete: return "Incomplete";
        case VmbFrameStatusTooSmall:   return "Too small";
        case VmbFrameStatusInvalid:    return "Invalid";
        default:                       return "unknown frame status";
    }
}

void frame_observer::check_frame_id( const AVT::VmbAPI::FramePtr& frame )
{
    VmbUint64_t frame_id;

    VmbErrorType status = frame->GetFrameID( frame_id );
    if( status == VmbErrorSuccess )
    {
        if( last_frame_id_ != 0 )
        {
            VmbUint64_t missing_frames = frame_id - last_frame_id_ - 1;
            if( missing_frames > 0 )
            {
                std::cerr << "Warning: " << missing_frames << " missing frame"
                          << ( missing_frames == 1 ? "" : "s" )
                          << " detected" << std::endl;
            }
        }
        last_frame_id_ = frame_id;
    }
    else
    {
        write_error( "Unable to get frame id", status );
    }
}

void frame_observer::check_frame_status( const AVT::VmbAPI::FramePtr& frame )
{
    VmbFrameStatusType frame_status;
    
    VmbErrorType status = frame->GetReceiveStatus( frame_status );
    if( status == VmbErrorSuccess )
    {
        if( frame_status != VmbFrameStatusComplete )
        {
            VmbUint32_t height;
            SP_ACCESS( frame )->GetHeight( height );

            VmbUint32_t width;
            SP_ACCESS( frame )->GetWidth( width );

            VmbUchar_t* image_buffer;
            SP_ACCESS( frame )->GetImage( image_buffer );

            VmbUint32_t size;
            SP_ACCESS( frame )->GetImageSize( size );

            VmbUint64_t frame_id;
            SP_ACCESS( frame )->GetFrameID( frame_id );

            std::cerr << "Warning: frame " << frame_id << " status "
                      << frame_status_to_string( frame_status )
                      << " (" << width << "x" << height << " with " << size << " bytes)" << std::endl;
        }
    }
}

} } // namespace snark { namespace vimba {
