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

/// @author andrew hill
/// @author vsevolod vlaskine (v.vlaskine@acfr.usyd.edu.au)

#include <boost/array.hpp>
#include <boost/crc.hpp>
#include "packet.h"

namespace snark { namespace sick { namespace cola { namespace binary {

static const boost::array< char, 4 > sentinel_ = {{ 0x02, 0x02, 0x02, 0x02 }};

// static const char* command_type_read_by_name = "sRN";
// static const char* response_type_read_by_name = "sRA";
// static const char* command_type_write_by_name = "sWN";
// static const char* response_type_write_by_name = "sWA";
// static const char* command_type_method = "sMN";
// static const char* response_type_method = "sAN";
// static const char* command_type_event = "sEN";
// static const char* response_type_event = "sEA";

bool header::valid() const { return ::memcmp( sentinel.data(), &sentinel_[0], sentinel_.size() ) == 0; }

std::string header::type() const
{
    const unsigned int command_type_size = 4; // quick and dirty
    const char* begin = reinterpret_cast< const char* >( this ) + header::size + command_type_size;
    const char* end = begin + length() - command_type_size;
    const char* p = begin;
    for( ; *p != ' ' && p < end; ++p );
    return std::string( begin, p - begin );
}

std::string header::type( const char* packet ) { return reinterpret_cast< const header* >( packet )->type(); }

} } } } // namespace snark { namespace sick { namespace cola { namespace binary {
