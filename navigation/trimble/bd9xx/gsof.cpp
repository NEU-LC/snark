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

#include <memory>
#include <comma/base/exception.h>
#include "gsof.h"

namespace snark { namespace trimble { namespace bd9xx { namespace gsof {

transmission::transmission() { records_.reserve( bd9xx::packet::size() * 10 ); } // quick and dirty: up to 10 packets per transmission according to documentation
    
void transmission::append( const char* buf, unsigned int size )
{
    if( complete() ) { COMMA_THROW( comma::exception, "cannot append to a complete transmission" ); }
    if( size < header_t_::size ) {  }
    const header_t_* header = reinterpret_cast< const header_t_* >( buf );
    if( header_ && header_->transmission_number() != header->transmission_number() ) { COMMA_THROW( comma::exception, "expected transmission number " << static_cast< unsigned int >( header_->transmission_number() ) << "; got: " << header->transmission_number() ); }
    if( header_ && header_->max_page_index() != header->max_page_index() ) { COMMA_THROW( comma::exception, "expected max page index" << static_cast< unsigned int >( header_->max_page_index() ) << "; got: " << header->max_page_index() ); }
    if( header_ && header_->page_index() + 1 != header->page_index() ) { COMMA_THROW( comma::exception, "expected page index" << static_cast< unsigned int >( header_->page_index() + 1 ) << "; got: " << header->page_index() ); }
    header_ = *header;
    char* p = &records_[ records_.size() ];
    unsigned int s = size - header_t_::size;
    records_.resize( records_.size() + s );
    ::memcpy( p, buf + header_t_::size, s );
}

bool transmission::complete() const { return header_ && header_->page_index() == header_->max_page_index(); } 

const std::vector< char >& transmission::records() const { return records_; }
        
} } } } // namespace snark { namespace trimble { namespace bd9xx { namespace gsof {
