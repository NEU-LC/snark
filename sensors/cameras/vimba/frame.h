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

#ifndef SNARK_SENSORS_VIMBA_FRAME_H_
#define SNARK_SENSORS_VIMBA_FRAME_H_

#include <VimbaCPP/Include/Frame.h>

namespace snark { namespace vimba {

class frame
{
    public:
        struct pixel_format_desc
        {
            pixel_format_desc( int type_, float width_adjustment_ )
                : type( type_ )
                , width_adjustment( width_adjustment_ )
            {}
            int type;
            float width_adjustment;
        };

        frame( const AVT::VmbAPI::FramePtr& frame_ptr );

        VmbUint64_t        id() const { return frame_id_; }
        VmbFrameStatusType status() const { return frame_status_; }
        const char*        status_as_string() const;
        VmbUint32_t        height() const { return height_; }
        VmbUint32_t        width() const { return width_; }
        VmbUint32_t        size() const { return size_; }
        VmbUchar_t*        image_buffer() const { return image_buffer_; }
        VmbPixelFormatType pixel_format() const { return pixel_format_; }
        VmbUint64_t        timestamp() const { return timestamp_; }
        pixel_format_desc  format_desc() const;

    private:
        VmbUint64_t        frame_id_;
        VmbFrameStatusType frame_status_;
        VmbUint32_t        height_;
        VmbUint32_t        width_;
        VmbUint32_t        size_;
        VmbUchar_t*        image_buffer_;
        VmbPixelFormatType pixel_format_;
        VmbUint64_t        timestamp_;
};

} } // namespace snark { namespace vimba {

#endif // SNARK_SENSORS_VIMBA_FRAME_H_
