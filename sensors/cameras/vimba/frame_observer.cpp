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
#include "frame.h"
#include "frame_observer.h"

namespace snark { namespace vimba {

void frame_observer::FrameReceived( const AVT::VmbAPI::FramePtr frame_ptr )
{
    frame frame( frame_ptr );

    frame.check_id();
    frame.check_status();

    pixel_format_desc fd = format_desc( frame.pixel_format() );

    cv::Mat cv_mat( frame.height()
                  , frame.width() * fd.width_adjustment
                  , fd.type
                  , frame.image_buffer() );

    serialization_->write( std::cout
                         , std::make_pair( boost::posix_time::microsec_clock::universal_time(), cv_mat ));

    m_pCamera->QueueFrame( frame_ptr );
}

frame_observer::pixel_format_desc frame_observer::format_desc( VmbPixelFormatType pixel_format ) const
{
    switch( pixel_format )
    {
        // Run vimba-cat --list-attributes --verbose to see all allowed formats for a given camera
        //
        // Below are the formats listed for the Prosilica GT3300. However,
        // actually trying them shows that many don't work. They are marked below.

        case VmbPixelFormatBayerGR12Packed: // BayerGR12Packed maps to VmbPixelFormatBayerGB12Packed
        case VmbPixelFormatBayerRG12Packed: // BayerRG12Packed (fails to set)
        case VmbPixelFormatBayerGB12Packed: // BayerGB12Packed (fails to set)
        case VmbPixelFormatBayerBG12Packed: // BayerBG12Packed (fails to set)
            return { CV_8UC1, 1.5 };

        case VmbPixelFormatMono8:       // Mono8
        case VmbPixelFormatBayerGR8:    // BayerGR8
        case VmbPixelFormatBayerRG8:    // BayerRG8 (fails to set)
        case VmbPixelFormatBayerBG8:    // BayerGB8 (fails to set)
            return { CV_8UC1, 1.0 };

        case VmbPixelFormatRgb8:        // RGB8Packed
        case VmbPixelFormatBgr8:        // BGR8Packed
            return { CV_8UC3, 1.0 };

        case VmbPixelFormatRgba8:       // RGBA8Packed
        case VmbPixelFormatBgra8:       // BGRA8Packed
            return { CV_8UC4, 1.0 };

        case VmbPixelFormatMono10:      // Mono10 (fails to set)
        case VmbPixelFormatMono12:      // Mono12 (fails to set)
        case VmbPixelFormatMono12Packed:// Mono12Packed (fails to set)
        case VmbPixelFormatMono14:      // Mono14 (fails to set)

        case VmbPixelFormatBayerBG10:   // BayerBG10 (fails to set)
        case VmbPixelFormatBayerGR12:   // BayerGR12
        case VmbPixelFormatBayerRG12:   // BayerRG12 (fails to set)

                                        // RGB10Packed (fails to set, no obvious mapping)
        case VmbPixelFormatRgb12:       // RGB12Packed (fails to set)

        case VmbPixelFormatYuv411:      // YUV411Packed
        case VmbPixelFormatYuv422:      // YUV422Packed
        case VmbPixelFormatYuv444:      // YUV444Packed

        default:
            COMMA_THROW( comma::exception, "unsupported format " << std::hex << pixel_format );
    }
}

} } // namespace snark { namespace vimba {
