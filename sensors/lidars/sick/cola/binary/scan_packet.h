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

#ifndef SNARK_SENSORS_SICK_COLA_BINARY_SCAN_PACKET_H_
#define SNARK_SENSORS_SICK_COLA_BINARY_SCAN_PACKET_H_

#include "packet.h"

namespace snark { namespace sick { namespace cola { namespace binary {

// todo: float32 values are not correctly parsed (I think endianness is backwards)

struct scan_packet
{
    enum { type_field_size = 11 };
    static const char* type() { return "LMDscandata"; }
    static const char* command_type() { return "sRA"; } // can also be sSN? typo?

    struct version_t : public comma::packed::packed_struct< version_t, 2 >
    {
        comma::packed::net_uint16 version;

        const char* end() const { return reinterpret_cast< const char* >( this ) + size; }
    };

    struct device_t : public comma::packed::packed_struct< device_t, 8 >
    {
        comma::packed::net_uint16 device_number; // user-defined in SOPAS
        comma::packed::net_uint32 serial_number;
        comma::packed::net_uint16 status; // 0=ok, 1=error, 2=pollution warning, 3=pollution error
        // warning: manual says status is 2xUINT8 but not specific on purpose of high-byte

        const char* end() const { return reinterpret_cast< const char* >( this ) + size; }
    };

    struct status_info_t : public comma::packed::packed_struct< status_info_t, 18 >
    {
        comma::packed::net_uint16 telegram_counter;
        comma::packed::net_uint16 scan_counter;
        comma::packed::net_uint32 time_since_boot;
        comma::packed::net_uint32 time_of_transmission;
        comma::packed::net_uint16 digital_input_state; // 0=all low, 3=all high
        // warning: manual says digital_input_state is 2xUINT8 but not specific on purpose of high-byte
        comma::packed::net_uint16 digital_output_state; // 0=all low, 7=all high
        // warning: manual says digital_output_state is 2xUINT8 but not specific on purpose of high-byte
        comma::packed::net_uint16 reserved;

        const char* end() const { return reinterpret_cast< const char* >( this ) + size; }
    };

    struct frequency_t : public comma::packed::packed_struct< frequency_t, 8 >
    {
        comma::packed::net_uint32 scan_frequency; // units 1/100 Hz
        comma::packed::net_uint32 measurement_frequency; // inverse of time between two shots in units of 100 Hz

        const char* end() const { return reinterpret_cast< const char* >( this ) + size; }
    };

    struct encoder_t : public comma::packed::packed_struct< encoder_t, 6 >
    {
        // stupid sick: the documentation lists these as both 16-bit values, yet 6 bytes are provided
        // variables will not be named to avoid possible misuse/confusion
        comma::packed::net_uint16 poorly_documented_value1;
        comma::packed::net_uint16 poorly_documented_value2;
        comma::packed::net_uint16 poorly_documented_value3;

        const char* end() const { return reinterpret_cast< const char* >( this ) + size; }
    };

    struct encoders_info_t : public comma::packed::packed_struct< encoders_info_t, 2 >
    {
        comma::packed::net_uint16 encoders_size;

        const encoder_t* encoders_begin() const;

        const encoder_t* encoders_end() const;

        const char* end() const;
    };

    struct channel16_t : public comma::packed::packed_struct< channel16_t, 21 >
    {
        comma::packed::string< 5 > channel_content; // "DIST1", "DIST2", "RSSI1", "RSSI2", or similar
        comma::packed::float32 scale_factor; // todo: this doens't work, actual value 0x3F800000 should read 1 (endiannness?)
        comma::packed::float32 scale_offset; // todo: this wouldn't work if it was non-zero
        comma::packed::net_int32 start_angle;
        comma::packed::net_uint16 steps;
        comma::packed::net_uint16 data_size;

        const comma::packed::net_uint16* data_begin() const;

        const comma::packed::net_uint16* data_end() const;

        const char* end() const;
    };

    struct channels16_t : public comma::packed::packed_struct< channels16_t, 2 >
    {
        comma::packed::net_uint16 channels_size;

        const channel16_t* channels_begin() const;

        const channel16_t* channels_next( const channel16_t* c = NULL ) const;

        const channel16_t* channels_end() const;

        const char* end() const;
    };

    struct channel8_t : public comma::packed::packed_struct< channel8_t, 21 >
    {
        comma::packed::string< 5 > channel_content; // "DIST1", "DIST2", "RSSI1", "RSSI2", or similar
        comma::packed::float32 scale_factor; // stupid sick: the manual says the type of these is "Real"...?!
        comma::packed::float32 scale_offset; // stupid sick: the manual says the type of these is "Real"...?!
        comma::packed::net_int32 start_angle;
        comma::packed::net_uint16 steps;
        comma::packed::net_uint16 data_size;

        const comma::packed::net_uint16* data_begin() const;

        const comma::packed::net_uint16* data_end() const;

        const char* end() const;
    };

    struct channels8_t : public comma::packed::packed_struct< channels8_t, 2 >
    {
        comma::packed::net_uint16 channels_size;

        const channel8_t* channels_begin() const;

        const channel8_t* channels_next( const channel8_t* c = NULL ) const;

        const channel8_t* channels_end() const;

        const char* end() const;
    };

    struct position_t : public comma::packed::packed_struct< position_t, 25 >
    {
        comma::packed::float32 x_position; // stupid sick: the manual says the type of these is "Real"...?!
        comma::packed::float32 y_position; // stupid sick: the manual says the type of these is "Real"...?!
        comma::packed::float32 z_position; // stupid sick: the manual says the type of these is "Real"...?!
        comma::packed::float32 x_rotation; // stupid sick: the manual says the type of these is "Real"...?!
        comma::packed::float32 y_rotation; // stupid sick: the manual says the type of these is "Real"...?!
        comma::packed::float32 z_rotation; // stupid sick: the manual says the type of these is "Real"...?!
        comma::packed::byte rotation_type; // 0=none, 1=pitch, 2=rollin, 3=free
        // stupid sick: the manual specifies another field but it looks like a mistake from the next section of the packet
        //comma::packed::byte name_present; // 0 = no name data, 1 = has name data

        const char* end() const { return reinterpret_cast< const char* >( this ) + size; }
    };

    struct position_info_t : public comma::packed::packed_struct< position_info_t, 2 >
    {
        comma::packed::net_uint16 data_present; // 0=no, 1=yes

        const position_t* position() const;

        const position_t* position_end() const;

        const char* end() const;
    };

    struct name_t : public comma::packed::packed_struct< name_t, 2 >
    {
        comma::packed::byte zero1;
        comma::packed::byte name_length; // should be limited to 0-16

        const comma::packed::byte* name_begin() const;

        const comma::packed::byte* name_end() const;

        const char* end() const;
    };

    struct name_info_t : public comma::packed::packed_struct< position_info_t, 2 >
    {
        comma::packed::net_uint16 data_present; // 0=no, 1=yes

        const name_t* name() const;

        const name_t* name_end() const;

        const char* end() const;
    };

    // todo: exactly the same as name_t, name_info_t ?
    struct comment_t : public comma::packed::packed_struct< comment_t, 1 >
    {
        comma::packed::byte comment_length; // should be limited to 0-16

        const comma::packed::byte* comment_begin() const;

        const comma::packed::byte* comment_end() const;

        const char* end() const;

    };

    struct comment_info_t : public comma::packed::packed_struct< comment_info_t, 2 >
    {
        comma::packed::net_uint16 data_present; // 0=no, 1=yes

        const comment_t* comment() const;

        const comment_t* comment_end() const;

        const char* end() const;
    };

    struct timestamp_t : public comma::packed::packed_struct< timestamp_t, 11 >
    {
        comma::packed::net_uint16 year;
        comma::packed::byte month;
        comma::packed::byte day;
        comma::packed::byte hour;
        comma::packed::byte minute;
        comma::packed::byte second;
        comma::packed::net_uint32 microseconds;

        const char* end() const { return reinterpret_cast< const char* >( this ) + size; }
    };

    struct timestamp_info_t : public comma::packed::packed_struct< timestamp_info_t, 2 >
    {
        comma::packed::net_uint16 data_present; // 0=no, 1=yes

        const timestamp_t* timestamp() const;

        const timestamp_t* timestamp_end() const;

        const char* end() const;
    };

    struct event_t : public comma::packed::packed_struct< event_t, 16 >
    {
        comma::packed::string< 4 > event_type; // e.g. "FDIN"
        comma::packed::net_uint32 encoder_position;
        comma::packed::net_uint32 time_of_event;
        comma::packed::net_uint32 angle_of_event;

        const char* end() const { return reinterpret_cast< const char* >( this ) + size; }
    };

    struct event_info_t : public comma::packed::packed_struct< event_info_t, 2 >
    {
        comma::packed::net_uint16 data_present; // 0=no, 1=yes

        const event_t* event() const;

        const event_t* event_end() const;

        const char* end() const;
    };

    scan_packet() {}

    scan_packet( const char* buffer ) : buffer( buffer ) {}

    typedef cola::binary::body_header< type_field_size > body_header_t;

    const char* body() const;

    const char* payload() const;

    const char* body_end() const;

    const cola::binary::header& header() const;

    const body_header_t& body_header() const;

    const version_t& version() const;

    const device_t& device() const;

    const status_info_t& status() const;

    const frequency_t& frequency() const;

    const encoders_info_t& encoders() const;

    const channels16_t& channels16() const;

    const channels8_t& channels8() const;

    const position_info_t& position() const;

    const name_info_t& name() const;

    const comment_info_t& comment() const;

    const timestamp_info_t& time() const;

    const event_info_t& event() const;

    char crc() const;

    bool valid() const;

    const char* buffer;

};

} } } } // namespace snark {  namespace sick { namespace cola { namespace binary {

#endif // #ifndef SNARK_SENSORS_SICK_COLA_BINARY_SCAN_PACKET_H_
