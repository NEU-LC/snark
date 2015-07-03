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

/// @author vsevolod vlaskine

#include "packet.h"

namespace snark { namespace trimble { namespace bd9xx {
    
unsigned char packet::checksum() const
{
    unsigned char sum = header().checksum();
    const char* begin = body();
    const char* end = begin + static_cast< unsigned int >( header().length() );
    for( const char* p = begin; p < end; sum += *p++ );
    return sum;
}

const bd9xx::header& packet::header() const { return *reinterpret_cast< const bd9xx::header* >( this ); }

bd9xx::header& packet::header() { return *reinterpret_cast< bd9xx::header* >( this ); }

const bd9xx::trailer& packet::trailer() const { return *reinterpret_cast< const bd9xx::trailer* >( body() + header().length() ); }

bd9xx::trailer& packet::trailer() { return *reinterpret_cast< bd9xx::trailer* >( body() + header().length() ); }

const char* packet::body() const { return &( this->operator[]( bd9xx::header::size ) ); }

char* packet::body() { return &( this->operator[]( bd9xx::header::size ) ); }

bool packet::valid() const { return trailer().checksum() == checksum(); }
    
} } } // namespace snark { namespace trimble { namespace bd9xx {
