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

#ifndef SNARK_GRAPHICS_APPLICATIONS_VIEWPOINTS_STREAMDATASOURCE_H_
#define SNARK_GRAPHICS_APPLICATIONS_VIEWPOINTS_STREAMDATASOURCE_H_

#include <string>
#include "snark/graphics/qt3d/qt3d_v2/data_source.h"
#include "snark/graphics/block_buffer.h"

namespace snark { namespace graphics { namespace view {

class stream_data_source : public qt3d::data_source
{
    public:
        stream_data_source( std::string input_filename );
        virtual ~stream_data_source() {}

        const qt3d::vertex_t* data() const { return buffer_.values().data(); }
        int count() const { return buffer_.size(); }

    private:
        block_buffer< qt3d::vertex_t > buffer_;
};

} } } // namespace snark { namespace graphics { namespace view {

#endif /*SNARK_GRAPHICS_APPLICATIONS_VIEWPOINTS_STREAMDATASOURCE_H_*/
