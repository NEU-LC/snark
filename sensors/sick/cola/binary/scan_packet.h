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

#ifndef SNARK_SENSORS_SICK_COLA_BINARY_SCAN_PACKET_H_
#define SNARK_SENSORS_SICK_COLA_BINARY_SCAN_PACKET_H_

#include "./packet.h"

namespace snark { namespace sick { namespace cola { namespace binary {

struct scan_packet
{
    enum { type_field_size = 11 };
    static const char* type() { return "LMDscandata"; }
    static const char* command_type() { return "sRA"; } // can also be sSN? typo?
    
    struct device_t : public comma::packed::packed_struct< device_t, 8 >
    {
        comma::packed::uint16 device_number; // user-defined in SOPAS
        comma::packed::uint32 serial_number;
        comma::packed::byte zero1; // stupid sick: apparently half of device status, but always zero
        comma::packed::byte status; // 0=ok, 1=error, 2=pollution warning, 3=pollution error        
    };
    
    struct status_info_t : public comma::packed::packed_struct< status_info_t, 18 >
    {
        comma::packed::uint16 telegram_counter;
        comma::packed::uint16 scan_counter;
        comma::packed::uint32 time_since_boot;
        comma::packed::uint32 time_of_transmission;
        comma::packed::byte zero2; // stupid sick: unused status byte
        comma::packed::byte digital_input_state; // 0=all low, 3=all high
        comma::packed::byte zero3; // stupid sick: unused status byte
        comma::packed::byte digital_output_state; // 0=all low, 7=all high
        comma::packed::uint16 reserved;        
    };
    
    struct frequency_t : public comma::packed::packed_struct< frequency_t, 8 >
    {
        comma::packed::uint32 scan_frequency;
        comma::packed::uint32 measurement_frequency;
    };
    
    struct encoder_t : public comma::packed::packed_struct< encoder_t, 4 >
    {
        comma::packed::uint16 position;
        comma::packed::uint16 speed;
    };
    
    struct channel_t : public comma::packed::packed_struct< encoder_t, 0 > // todo
    {
        comma::packed::string< 5 > channel_content; // "DIST1", "DIST2", "RSSI1", "RSSI2", or similar
        comma::packed::float32 scale_factor;
        comma::packed::float32 scale_offset;
        comma::packed::uint32 start_angle;
        comma::packed::uint16 steps;
        comma::packed::uint16 data_size;
        
        const comma::packed::uint16* data_begin() const { return reinterpret_cast< const comma::packed::uint16* >( reinterpret_cast< const char* >( this ) + size ); }
        
        const comma::packed::uint16* data_end() const { return data_begin() + size * sizeof( comma::packed::uint16 ); }
        
        const char* end() const { return reinterpret_cast< const char* >( data_end() ); }
    };
    
    struct channels_t : public comma::packed::packed_struct< encoder_t, 2 > // todo
    {
        comma::packed::uint16 channels_size;
        
        const channel_t* begin() const { return reinterpret_cast< const channel_t* >( reinterpret_cast< const char* >( this ) + sizeof( comma::packed::uint16 ) ); } // todo: throw, if channels_size == 0
        
        const channel_t* next( const channel_t* c = NULL ) { return NULL ? begin() : reinterpret_cast< const channel_t* >( c->end() ); }
        
        const char* end() const; // todo: iterate channels, find end
    };
    
    struct part_1_t : public comma::packed::packed_struct< part_1_t, 2 + device_t::size + status_info_t::size + frequency_t::size + 2 >
    {
        comma::packed::uint16 version;
        device_t device;
        status_info_t status_info;
        frequency_t frequency;
        comma::packed::uint16 encoders_size;
        
        const encoder_t* encoders() const { return reinterpret_cast< const encoder_t* >( reinterpret_cast< const char* >( this ) + size ); }
        
        const encoder_t* encoders_end() const { return reinterpret_cast< const encoder_t* >( end() ); }
        
        const char* end() const { return reinterpret_cast< const char* >( encoders() ) + encoder_t::size * encoders_size(); }
    };
    
    scan_packet() {}
    
    scan_packet( const char* buffer ) : buffer( buffer ) {}
    
    const cola::binary::header& header() const { return *reinterpret_cast< const cola::binary::header* >( buffer ); }
    
    const part_1_t& part_1() const { return *reinterpret_cast< const part_1_t* >( buffer + cola::binary::header::size + body_header< type_field_size >::size ); }
    
    const channels_t& channels() const { return *reinterpret_cast< const channels_t* >( part_1().end() ); }
    
    const char* buffer;
    
    // todo: the rest of the packet

    
    
    
/*
    // encoder info (2 + 4n; n=number of encoders)
    comma::packed::int16 number_of_encoders;
    // todo: variable packet... depends on value of number_of_encoders
    // todo: per encoder channel (I think, documentation is unclear)
        // comma::packed::uint16 encoder_position;
        // comma::packed::uint16 encoder_speed;
    comma::packed::uint16 num_output_channels_16bit; // # of 16-bit output channels (LMS1xx 1-2, LMS5xx 0 or 5, others depend on sectors)
    // todo: per 16-bit channel:
        // comma::packed::string< 5 > channel_content; // "DIST1", "DIST2", "RSSI1", "RSSI2", or similar
        // comma::packed::float scale_factor;
        // comma::packed::float scale_offset;
        // comma::packed::uint32 start_angle;
        // comma::packed::uint16 steps;
        // comma::packed::uint16 data_values;
        // todo: per shot:
            // comma::packed::uint16 data;
    comma::packed::uint16 num_output_channels_8bit; // # of 8-bit output channels
    // todo: per 8-bit channel
        // comma::packed::string< 5 > channel_content; // e.g. DIST1, RSSI1
        // comma::packed::int32 scale_factor;
        // comma::packed::int32 scale_offset;
        // comma::packed::int32 start_angle; // 1/10,000 deg
        // comma::packed::uint16 steps;
        // comma::packed::uint16 data_values;
        // todo: per data value:
            // comma::packed::uint8 data;
    comma::packed::uint16 position_data_present; // 0 if not, 1 if present
    // todo: if present
    // comma::packed::int32 position_x; // "Real" type, units unclear
    // comma::packed::int32 position_y;
    // comma::packed::int32 position_z;
    // comma::packed::int32 rotation_x;
    // comma::packed::int32 rotation_y;
    // comma::packed::int32 rotation_z;
    // comma::packed::uint8 rotation_type; // enum 0=none, 1=pitch, 2=rollin, 3=free
    // comma::packed::uint8 transmits_name_of_device; // actually i supsect this is a mistake in the documentation
    comma::packed::uint16 name_data_present; // 0 if not, 1 if present
    comma::packed::uint8 name_length;
    // comma::packed::string < name_length > name; // up to 16 chars
    comma::packed::uint16 comment_data_present; // 0 if not, 1 if present
    comma::packed::uint8 comment_length;
    // comma::packed::string < comment_length > comment; // up to 16 chars
    comma::packed::uint16 time_data_present; // 0 if not, 1 if present
    comma::packed::uint16 time_year; // 0 to 0x270F = 9999
    comma::packed::uint8 time_month; // 0-12 stupid sick: may be 0-11 or 1-12
    comma::packed::uint8 time_day; // 0-31
    comma::packed::uint8 time_hour; // 0-23
    comma::packed::uint8 time_minute; // 0-59
    comma::packed::uint8 time_second; // 0-59
    comma::packed::uint32 time_microseconds; // 0 to 999,999 (not total_microseconds, just added onto datetime)
    comma::packed::uint16 event_info_present; // 0=none, 1=present
    // comma::packed::string< 4 > fdin; // FDIN not sure if this is the value or will actually be a variable name
    // comma::packed::uint32 encoder_position;
    // comma::packed::uint32 event_time; // in microseconds, probably based on the rolling counter, but I'm pretty sure that's actually a millisecond counter
    // comma::packed::int32 event_angle; // encoder angle at time of event*/
};

} } } } // namespace snark {  namespace sick { namespace cola { namespace binary {

#endif // #ifndef SNARK_SENSORS_SICK_COLA_BINARY_SCAN_PACKET_H_
