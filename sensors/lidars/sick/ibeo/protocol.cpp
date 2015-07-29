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


#include <boost/optional.hpp>
#include <comma/base/exception.h>
#include "protocol.h"

namespace snark {  namespace sick { namespace ibeo {

class protocol::impl
{
    public:
        impl( std::iostream& stream )
            : m_istream( &stream )
            , m_ostream( &stream )
            , m_buf( 1024 ) // arbitrary
            , m_header( reinterpret_cast< const header* >( &m_buf[0] ) )
            , m_payload( &m_buf[0] + header::size )
            , m_synchronized( false )
        {
        }

        impl( std::istream& stream )
            : m_istream( &stream )
            , m_ostream( NULL )
            , m_buf( 1024 ) // arbitrary
            , m_header( reinterpret_cast< const header* >( &m_buf[0] ) )
            , m_payload( &m_buf[0] + header::size )
            , m_synchronized( false )
        {
        }

        void reset_dsp()
        {
            m_commandId.reset();
            sendcommand( commands::reset_dsp() );
        }

        template < typename command >
        typename command::response write( const command& c )
        {
            if( m_fault ) { COMMA_THROW( comma::exception, "got command, while having uncleared fault" ); }
            sendcommand( c );
            m_commandId = static_cast< commands::types >( command::id );
            return readresponse< command >();
        }

        const scan_packet* readscan()
        {
            if( m_fault ) { COMMA_THROW( comma::exception, "asked to read scan, while having uncleared fault" ); }
            if( m_commandId ) { COMMA_THROW( comma::exception, "cannot read scan, while waiting for response to 0x" << std::hex << *m_commandId << std::dec ); }
            if( !readpacket() ) { return NULL; }
            switch( m_header->type() )
            {
                case header::scan_type:
                    return reinterpret_cast< const scan_packet* >( &m_buf[0] );
                case header::fault_type:
                    throw faultException(); // COMMA_THROW( comma::exception, "received fault, while reading scan" );
                case header::response_type:
                    COMMA_THROW( comma::exception, "expected scan data, got command response of type 0x" << std::hex << ( reinterpret_cast< const commands::response_header* >( m_payload )->id() & 0x3fff ) << std::dec );
                default:
                    COMMA_THROW( comma::exception, "expected scan data, got packet of unknown type (0x" << std::hex << m_header->type() << std::dec );
            }
        }

        boost::optional< fault > last_fault()
        {
            boost::optional< fault > f = m_fault;
            m_fault.reset();
            return f;
        }

    private:
        std::istream* m_istream;
        std::ostream* m_ostream;
        boost::optional< commands::types > m_commandId;
        boost::optional< fault > m_fault;
        std::vector< char > m_buf;
        const header* m_header;
        char* m_payload;
        bool m_synchronized;

        template < typename command >
        void sendcommand( const command& c )
        {
            if( !m_ostream ) { COMMA_THROW( comma::exception, "cannot write to read-only stream" ); }
            if( m_commandId ) { COMMA_THROW( comma::exception, "got a new command (0x" << std::hex << command::id << "), while waiting for response to 0x" << *m_commandId << std::dec ); }
            commands::packet< command > packet( c );
            m_ostream->write( packet.data(), commands::packet< command >::size );
            m_ostream->flush();
            if( m_ostream->bad() ) { COMMA_THROW( comma::exception, "failed to send command (0x" << std::hex << command::id << std::dec ); }
        }

        template < typename command >
        typename command::response readresponse()
        {
            while( true )
            {
                if( !readpacket() ) { COMMA_THROW( comma::exception, "expected command response, got end of stream" ); }
                switch( m_header->type() )
                {
                    case header::scan_type:
                    case header::fault_type: // cannot throw yet, since need to get response first
                        break;
                    case header::response_type:
                    {
                        unsigned int id = reinterpret_cast< const commands::response_header* >( m_payload )->id() & 0x3fff;
                        if( int( id ) != *m_commandId ) { COMMA_THROW( comma::exception, "expected response to command 0x" << std::hex << *m_commandId << ", got 0x" << id << std::dec ); }
                        m_commandId.reset();
                        return *( reinterpret_cast< typename command::response* >( m_payload ) );
                    }
                    default:
                        COMMA_THROW( comma::exception, "expected command response, got packet of unknown type (0x" << std::hex << m_header->type() << std::dec );
                }
            }
        }

        bool readpacket() // watch performance, if bad, optimize with chunk reading
        {
            std::size_t offset = 0;
            if( !m_synchronized )
            {
                bool first = true;
                while( offset < 4 ) // watch performance, if bad, optimize with chunk reading
                {
                    m_istream->read( &m_buf[offset], 1 );
                    if( first && m_istream->eof() ) { return false; }
                    first = false;
                    if( m_istream->eof() || m_istream->bad() ) { COMMA_THROW( comma::exception, "failed to synchronize, bad stream" ); }
                    unsigned char header_byte = reinterpret_cast<const unsigned char*>( m_header->data() )[offset];
                    offset = header_byte == header::sentinel_value[offset] ? offset + 1 : 0;
                }
                m_synchronized = true; // quick and dirty: synchronize once, should be enough
            }
            m_istream->read( &m_buf[0] + offset, header::size - offset );
            if( m_istream->gcount() == 0 ) { return false; }
            if( m_istream->eof() || m_istream->bad() || m_istream->gcount() != static_cast< int >( header::size - offset ) ) { COMMA_THROW( comma::exception, "failed to read packet header" ); }
            if( !m_header->valid() )
            {
                m_synchronized = false;
                COMMA_THROW( comma::exception, "invalid header (stream from laser went out of sync)" );
            }
            const std::size_t size = m_header->payload_size();
            if( m_buf.size() < size + header::size )
            {
                m_buf.resize( size + header::size );
                m_header = reinterpret_cast< const header* >( &m_buf[0] );
                m_payload = &m_buf[0] + header::size;
            }
            m_istream->read( m_payload, size ); // todo: check payload size so that it does not go crazy?
            if( m_istream->eof() ) { COMMA_THROW( comma::exception, "failed to read payload of size " << m_header->payload_size() << ", end of file" ); }
            if( m_istream->bad() || m_istream->gcount() != int( size ) ) { COMMA_THROW( comma::exception, "failed to read payload of size " << m_header->payload_size() << ", bad stream" ); }
            if( m_header->type() == header::fault_type ) { m_fault = *( reinterpret_cast< fault* >( m_payload ) ); }
            return true;
        }
};

protocol::protocol( std::iostream& stream ) : m_pimpl( new impl( stream ) ) {}

protocol::protocol( std::istream& stream ) : m_pimpl( new impl( stream ) ) {}

protocol::~protocol() { delete m_pimpl; }

void protocol::reset_dsp() { m_pimpl->reset_dsp(); }

template commands::reset::response protocol::write< commands::reset >( const commands::reset& command );
template commands::get_status::response protocol::write< commands::get_status >( const commands::get_status& command );
template commands::save_configuration::response protocol::write< commands::save_configuration >( const commands::save_configuration& command );
template commands::get::response protocol::write< commands::get >( const commands::get& command );
template commands::set::response protocol::write< commands::set >( const commands::set& command );
template commands::start::response protocol::write< commands::start >( const commands::start& command );
template commands::stop::response protocol::write< commands::stop >( const commands::stop& command );
template commands::set_ntp_seconds::response protocol::write< commands::set_ntp_seconds >( const commands::set_ntp_seconds& command );
template commands::set_ntp_fractions::response protocol::write< commands::set_ntp_fractions >( const commands::set_ntp_fractions& command );

template < typename command >
typename command::response protocol::write( const command& c ) { return m_pimpl->write( c ); }

const ibeo::scan_packet* protocol::readscan() { return m_pimpl->readscan(); }

boost::optional< fault > protocol::last_fault() { return m_pimpl->last_fault(); }

} } } // namespace snark {  namespace sick { namespace ibeo {
