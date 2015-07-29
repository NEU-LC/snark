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

#include "scan_packet.h"
#include <comma/base/exception.h>

// todo: templates for channels16_t + channels8_t?
// todo: general template for all sections with optional components?

namespace snark { namespace sick { namespace cola { namespace binary {

// encoders_info_t
const scan_packet::encoder_t* scan_packet::encoders_info_t::encoders_begin() const
{
    if ( encoders_size() == 0 ) { COMMA_THROW( comma::exception, "no encoder data" ); }
    return reinterpret_cast< const scan_packet::encoder_t* >( reinterpret_cast< const char* >( this ) + size );
}

const scan_packet::encoder_t* scan_packet::encoders_info_t::encoders_end() const
{
    return reinterpret_cast< const scan_packet::encoder_t* >( end() );
}

const char* scan_packet::encoders_info_t::end() const
{
    return reinterpret_cast< const char* >( this ) + size + encoders_size() * encoder_t::size;
}

// channel16_t
const comma::packed::net_uint16* scan_packet::channel16_t::data_begin() const
{
    if ( data_size() == 0 ) { COMMA_THROW( comma::exception, "no channel data in 16-bit channel " << channel_content() ); }
    return reinterpret_cast< const comma::packed::net_uint16* >( reinterpret_cast< const char* >( this ) + size );
}

const comma::packed::net_uint16* scan_packet::channel16_t::data_end() const
{
    return reinterpret_cast< const comma::packed::net_uint16* >( end() );
}

const char* scan_packet::channel16_t::end() const
{
    return reinterpret_cast< const char* >( this ) + size + data_size() * sizeof( comma::packed::net_uint16 );
}

// channels16_t
const scan_packet::channel16_t* scan_packet::channels16_t::channels_begin() const
{
    if ( channels_size() == 0 ) { COMMA_THROW( comma::exception, "no 16-bit channels" ); }
    return reinterpret_cast< const scan_packet::channel16_t* >( reinterpret_cast< const char* >( this ) + size );
}

const scan_packet::channel16_t* scan_packet::channels16_t::channels_next( const channel16_t* c ) const
{
    if ( c == NULL )
    {
        return reinterpret_cast< const channel16_t* >( reinterpret_cast< const char* >( this ) + size );
    }
    else
    {
        return reinterpret_cast< const scan_packet::channel16_t* >( c->end() );
    }
}

const scan_packet::channel16_t* scan_packet::channels16_t::channels_end() const
{
    const scan_packet::channel16_t* c = NULL;
    for(unsigned int channel = 0; channel < channels_size(); ++channel )
    {
        c = channels_next(c);
    }
    if ( c == NULL )
    {
        return reinterpret_cast< const scan_packet::channel16_t* >( reinterpret_cast< const char* >( this ) + size );
    }
    else
    {
        return reinterpret_cast< const scan_packet::channel16_t* >( c->end() );
    }
}

const char* scan_packet::channels16_t::end() const
{
    return reinterpret_cast< const char* >( channels_end() );
}

// channel8_t
const comma::packed::net_uint16* scan_packet::channel8_t::data_begin() const
{
    if ( data_size() == 0 ) { COMMA_THROW( comma::exception, "no channel data in 8-bit channel " << channel_content() ); }
    return reinterpret_cast< const comma::packed::net_uint16* >( reinterpret_cast< const char* >( this ) + size );
}

const comma::packed::net_uint16* scan_packet::channel8_t::data_end() const
{
    return reinterpret_cast< const comma::packed::net_uint16* >( end() );
}

const char* scan_packet::channel8_t::end() const
{
    return reinterpret_cast< const char* >( this ) + size + data_size() * sizeof( comma::packed::net_uint16 );
}

// channels8_t
const scan_packet::channel8_t* scan_packet::channels8_t::channels_begin() const
{
    if ( channels_size() == 0 ) { COMMA_THROW( comma::exception, "no 8-bit channels" ); }
    return reinterpret_cast< const scan_packet::channel8_t* >( reinterpret_cast< const char* >( this ) + size );
}

const scan_packet::channel8_t* scan_packet::channels8_t::channels_next( const channel8_t* c ) const
{
    if ( c == NULL )
    {
        return reinterpret_cast< const scan_packet::channel8_t* >( reinterpret_cast< const char* >( this ) + size );
    }
    else
    {
        return reinterpret_cast< const scan_packet::channel8_t* >( c->end() );
    }
}

const scan_packet::channel8_t* scan_packet::channels8_t::channels_end() const
{
    const scan_packet::channel8_t* c = NULL;
    for(unsigned int channel = 0; channel < channels_size(); ++channel )
    {
        c = channels_next(c);
    }
    if ( c == NULL )
    {
        return reinterpret_cast< const scan_packet::channel8_t* >( reinterpret_cast< const char* >( this ) + size );
    }
    else
    {
        return reinterpret_cast< const scan_packet::channel8_t* >( c->end() );
    }
}

const char* scan_packet::channels8_t::end() const
{
    return reinterpret_cast< const char* >( channels_end() );
}

// position_info_t
const scan_packet::position_t* scan_packet::position_info_t::position() const
{
    if ( data_present() == 0 ) { COMMA_THROW( comma::exception, "no position data" ); }
    return reinterpret_cast< const scan_packet::position_t* >( reinterpret_cast< const char* >( this ) + size );
}

const scan_packet::position_t* scan_packet::position_info_t::position_end() const
{
    // todo: is this necessary?
    return reinterpret_cast< const scan_packet::position_t* >( end() );
}

const char* scan_packet::position_info_t::end() const
{
    return data_present() == 1 ? position()->end() : reinterpret_cast< const char* >( this ) + size;
}

// name_t
const comma::packed::byte* scan_packet::name_t::name_begin() const
{
    if ( name_length() == 0 ) { COMMA_THROW( comma::exception, "name length is zero" ); }
    return reinterpret_cast< const comma::packed::byte* >( reinterpret_cast< const char* >( this ) + size );
}

const comma::packed::byte* scan_packet::name_t::name_end() const
{
    return reinterpret_cast< const comma::packed::byte* >( end() );
}

const char* scan_packet::name_t::end() const
{
    return reinterpret_cast< const char* >( this ) + size + name_length() * sizeof( comma::packed::byte );
}

// name_info_t
const scan_packet::name_t* scan_packet::name_info_t::name() const
{
    if ( data_present() == 0 ) { COMMA_THROW( comma::exception, "no name data" ); }
    return reinterpret_cast< const scan_packet::name_t* >( reinterpret_cast< const char* >( this ) + size );
}

const scan_packet::name_t* scan_packet::name_info_t::name_end() const
{
    return reinterpret_cast< const scan_packet::name_t* >( end() );
}

const char* scan_packet::name_info_t::end() const
{
    return data_present() == 1 ? name()->end() : reinterpret_cast< const char* >( this ) + size;
}

// comment_t
const comma::packed::byte* scan_packet::comment_t::comment_begin() const
{
    if ( comment_length() == 0 ) { COMMA_THROW( comma::exception, "comment length is zero" ); }
    return reinterpret_cast< const comma::packed::byte* >( reinterpret_cast< const char* >( this ) + size );
}

const comma::packed::byte* scan_packet::comment_t::comment_end() const
{
    return reinterpret_cast< const comma::packed::byte* >( this ) + size + comment_length() * sizeof( comma::packed::byte );
}

const char* scan_packet::comment_t::end() const
{
    return reinterpret_cast< const char* >( comment_end() );
}

// comment_info_t
const scan_packet::comment_t* scan_packet::comment_info_t::comment() const
{
    if ( data_present() == 0 ) { COMMA_THROW( comma::exception, "no comment data" ); }
    return reinterpret_cast< const scan_packet::comment_t* >( reinterpret_cast< const char* >( this ) + size );
}

const scan_packet::comment_t* scan_packet::comment_info_t::comment_end() const
{
    return reinterpret_cast< const scan_packet::comment_t* >( end() );
}

const char* scan_packet::comment_info_t::end() const
{
    return data_present() == 1 ? comment()->end() : reinterpret_cast< const char* >( this ) + size;
}

// timestamp_info_t
const scan_packet::timestamp_t* scan_packet::timestamp_info_t::timestamp() const
{
    if ( data_present() == 0 ) { COMMA_THROW( comma::exception, "no timestamp data" ); }
    return reinterpret_cast< const scan_packet::timestamp_t* >( reinterpret_cast< const char* >( this ) + size );
}

const scan_packet::timestamp_t* scan_packet::timestamp_info_t::timestamp_end() const
{
    // todo: is this necessary?
    return reinterpret_cast< const scan_packet::timestamp_t* >( end() );
}

const char* scan_packet::timestamp_info_t::end() const
{
    return data_present() == 1 ? timestamp()->end() : reinterpret_cast< const char* >( this ) + size;
}

// event_info_t
const scan_packet::event_t* scan_packet::event_info_t::event() const
{
    if ( data_present() == 0 ) { COMMA_THROW( comma::exception, "no event data" ); }
    return reinterpret_cast< const scan_packet::event_t* >( reinterpret_cast< const char* >( this ) + size );
}

const scan_packet::event_t* scan_packet::event_info_t::event_end() const
{
    // todo: is this necessary?
    return reinterpret_cast< const scan_packet::event_t* >( end() );
}

const char* scan_packet::event_info_t::end() const
{
    return data_present() == 1 ? event()->end() : reinterpret_cast< const char* >( this ) + size;
}

// progressive parsing of scan_packet
const char* scan_packet::body() const
{
    return reinterpret_cast< const char* > ( buffer + cola::binary::header::size );
}

const char* scan_packet::payload() const
{
    return body() + body_header_t::size;
}

const char* scan_packet::body_end() const
{
    return body() + header().length();
}

const cola::binary::header& scan_packet::header() const
{
    return *reinterpret_cast< const cola::binary::header* >( buffer );
}

const scan_packet::body_header_t& scan_packet::body_header() const
{
    return *reinterpret_cast< const scan_packet::body_header_t* >( body() );
}

const scan_packet::version_t& scan_packet::version() const
{
    return *reinterpret_cast< const scan_packet::version_t* >( payload() );
}

const scan_packet::device_t& scan_packet::device() const
{
    return *reinterpret_cast< const scan_packet::device_t* >( version().end() );
}

const scan_packet::status_info_t& scan_packet::status() const
{
    return *reinterpret_cast< const scan_packet::status_info_t* >( device().end() );
}

const scan_packet::frequency_t& scan_packet::frequency() const
{
    return *reinterpret_cast< const scan_packet::frequency_t* >( status().end() );
}

const scan_packet::encoders_info_t& scan_packet::encoders() const
{
    return *reinterpret_cast< const scan_packet::encoders_info_t* >( frequency().end() );
}

const scan_packet::channels16_t& scan_packet::channels16() const
{
    return *reinterpret_cast< const scan_packet::channels16_t* >( encoders().end() );
}

const scan_packet::channels8_t& scan_packet::channels8() const
{
    return *reinterpret_cast< const scan_packet::channels8_t* >( channels16().end() );
}

const scan_packet::position_info_t& scan_packet::position() const
{
    return *reinterpret_cast< const scan_packet::position_info_t* >( channels8().end() );
}

const scan_packet::name_info_t& scan_packet::name() const
{
    return *reinterpret_cast< const scan_packet::name_info_t* >( position().end() );
}

const scan_packet::comment_info_t& scan_packet::comment() const
{
    return *reinterpret_cast< const scan_packet::comment_info_t* >( name().end() );
}

const scan_packet::timestamp_info_t& scan_packet::time() const
{
    return *reinterpret_cast< const scan_packet::timestamp_info_t* >( comment().end() );
}

const scan_packet::event_info_t& scan_packet::event() const
{
    return *reinterpret_cast< const scan_packet::event_info_t* >( time().end() );
}

// crc
char scan_packet::crc() const
{
    return *body_end();
}

bool scan_packet::valid() const
{
    char checksum = 0;
    for ( const char* c = body(); c < body_end(); ++c ) { checksum ^= *c; }
    bool checksum_valid = checksum == crc();
    bool full_packed_parsed = event().end() == body_end();
    return checksum_valid && full_packed_parsed;
}


} } } } // namespace snark {  namespace sick { namespace cola { namespace binary {
