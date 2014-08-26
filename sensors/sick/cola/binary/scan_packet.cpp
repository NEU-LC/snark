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
// 3. All advertising materials mentioning features or use of this software
//    must display the following acknowledgement:
//    This product includes software developed by the The University of Sydney.
// 4. Neither the name of the The University of Sydney nor the
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

/// @author andrew hill
/// @author vsevolod vlaskine (v.vlaskine@acfr.usyd.edu.au)

#include "./scan_packet.h"

// todo: any executable code goes here

namespace snark { namespace sick { namespace cola { namespace binary {

const scan_packet::channel16_t* scan_packet::channels16_t::channels_end() const
{
    const scan_packet::channel16_t* c = NULL;
    for(unsigned int channel = 0; channel < *channels_size.data(); ++channel )
    {
        c = channels_next(c);
    }
    return c == NULL ? reinterpret_cast< const scan_packet::channel16_t* >( reinterpret_cast< const char* >( this ) + size ) : c;
}

// todo: templates for channels16_t and channels8_t
const scan_packet::channel8_t* scan_packet::channels8_t::channels_end() const
{
    const scan_packet::channel8_t* c = NULL;
    for(unsigned int channel = 0; channel < *channels_size.data(); ++channel )
    {
        c = channels_next(c);
    }
    return c == NULL ? reinterpret_cast< const scan_packet::channel8_t* >( reinterpret_cast< const char* >( this ) + size ) : c;
}

} } } } // namespace snark {  namespace sick { namespace cola { namespace binary {
