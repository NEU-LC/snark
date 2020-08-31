// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2020 Mission Systems Pty Ltd
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of the copyright owner nor the names of the contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
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

#pragma once

#include <bnet_interface.h>
#include <comma/csv/stream.h>
#include "echodyne.h"

namespace snark { namespace echodyne {

// thin wrapper around the bnet_interface API

class radar
{
public:
    radar();
    ~radar();

    void connect( const std::string& address="169.254.1.10", int port=23, const std::string& log_dir="." );
    void command( const std::string& cmd );
    void enable_buffer( mesa_data_t d_type ) { radar_->set_collect( d_type, true ); }
    void disable_buffer( mesa_data_t d_type ) { radar_->set_collect( d_type, false ); }
    template< typename O > void output( mesa_data_t d_type, comma::csv::binary_output_stream< O >& os );
    std::pair< unsigned int, unsigned int > get_time();
    void set_time();

private:
    std::pair< bool, std::string > send_command( const std::string& cmd, bool verbose );
    void set_time_offset( unsigned int days, unsigned int milliseconds );
    void show_buffer_states();
    void show_buffer_state( mesa_data_t d_type );
    void show_system_state();

    template< typename T > std::vector< T > output_packet();

    std::unique_ptr< bnet_interface > radar_;
    bool connected;
};

// a single packet from the echodyne API might end up as multiple entries in the comma world
template<> std::vector< status_data_t > radar::output_packet();
template<> std::vector< rvmap_data_t > radar::output_packet();
template<> std::vector< detection_data_t > radar::output_packet();
template<> std::vector< track_data_t > radar::output_packet();
template<> std::vector< meas_data_t > radar::output_packet();

template< typename T > void radar::output( mesa_data_t d_type, comma::csv::binary_output_stream< T >& os )
{
    int num_buffers = radar_->get_n_buffered( d_type );
    for( int i = 0; i < num_buffers; i++ )
    {
        std::vector< T > data = output_packet< T >();
        for( auto& d : data ) { os.write( d ); }
    }
    if( num_buffers > 0 ) { os.flush(); }
}

} } // namespace snark { namespace echodyne {
