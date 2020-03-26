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

#include <bnet_reader.h>
#include <comma/csv/stream.h>
#include "echodyne.h"

namespace snark { namespace echodyne {

// thin wrapper around the bnet_reader API

class reader
{
public:
    reader( const std::string& source_folder );
    ~reader() {}

    template< typename O > void output( mesa_data_t d_type, comma::csv::binary_output_stream< O >& os );

private:
    template< typename T > std::vector< T > output_packet();
    template< typename T > T get_data();

    std::unique_ptr< bnet_reader > reader_;
};

template<> MESAK_Status reader::get_data() { return reader_->get_data( STATUS_DATA ); }
template<> MESAK_Rvmap reader::get_data() { return reader_->get_data( RVMAP_DATA ); }
template<> MESAK_Detection reader::get_data() { return reader_->get_data( DETECTION_DATA ); }
template<> MESAK_Track reader::get_data() { return reader_->get_data( TRACK_DATA ); }
template<> MESAK_Measurement reader::get_data() { return reader_->get_data( MEAS_DATA ); }

template<> std::vector< status_data_t > reader::output_packet() { return from_packet( get_data< MESAK_Status >()); }
template<> std::vector< rvmap_data_t > reader::output_packet() { return from_packet( get_data< MESAK_Rvmap >()); }
template<> std::vector< detection_data_t > reader::output_packet() { return from_packet( get_data< MESAK_Detection >()); }
template<> std::vector< track_data_t > reader::output_packet() { return from_packet( get_data< MESAK_Track >()); }
template<> std::vector< meas_data_t > reader::output_packet() { return from_packet( get_data< MESAK_Measurement >()); }

template< typename T > void reader::output( mesa_data_t d_type, comma::csv::binary_output_stream< T >& os )
{
    while( !reader_->isEmpty( d_type ))
    {
        std::vector< T > data = output_packet< T >();
        for( auto& d : data ) { os.write( d ); }
    }
    os.flush();
}

} } // namespace snark { namespace echodyne {
