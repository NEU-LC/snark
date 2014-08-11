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

#ifndef SNARK_SENSORS_SICK_COLA_BINARY_PACKETS_H_
#define SNARK_SENSORS_SICK_COLA_BINARY_PACKETS_H_

#include <boost/array.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <comma/packed/byte.h>
#include <comma/packed/little_endian.h>
#include <comma/packed/big_endian.h>
#include <comma/packed/string.h>
#include <comma/packed/struct.h>

namespace snark { namespace sick { namespace cola { namespace binary {

struct header : public comma::packed::packed_struct< header, 8 >
{
    comma::packed::string< 4 > sentinel;
    comma::packed::uint32 length;

    bool valid() const;
    std::string type() const;
    static std::string type( const char* packet );
};

template < typename P >
struct body : public comma::packed::packed_struct< body< P >, 4 + P::type_field_size + 1 + P::size >
{
    comma::packed::string< 3 > command_type;
    comma::packed::const_byte< ' ' > space1;
    comma::packed::string< P::type_field_size, ' ' > type;
    comma::packed::const_byte< ' ' > space2;
    P payload;
};

template < typename P >
struct packet : public comma::packed::packed_struct< packet< P >, header::size + body< P >::size + 1 >
{
    binary::header header;
    binary::body< P > body;
    comma::packed::byte crc;

    bool valid() const;
};

namespace payloads {

// todo: some messages have optional components...
// todo: some messages aren't available in binary (it seems ascii is the native language and many binary messages are poorly adapted binary ones)

// todo: it seems SICK do actually respect consistent names between request & response, so can probably
// afford to simplify this somewhat
// todo: request/response pairs seem consistent: sMN:sAN, sRN:sRA, sWN:sWA, sEN:sEA
// todo: scan stream mode is the only exception I can see for request:response
// todo: some units used commonly: degrees represented as 1/100000, Hz as 1/100,

struct set_access_mode
{
    struct request : public comma::packed::packed_struct< request, 5 >
    {
        enum { type_field_size = 13 };
        static const char* type() { return "SetAccessMode"; }
        static const char* command_type() { return "sMN"; }
        comma::packed::byte user_level; // 2=maintenance, 3=authorised client, 4=service
        comma::packed::net_uint32 password_hash;
    };

    struct response : public comma::packed::packed_struct< response, 1 >
    {
        enum { type_field_size = 13 };
        static const char* type() { return "SetAccessMode"; }
        static const char* command_type() { return "sAN"; }
        comma::packed::byte status; // 0=error, 1=success
    };
};

struct set_frequency_and_angular_resolution
{
    struct request : public comma::packed::packed_struct< request, 18 >
    {
        enum { type_field_size = 14 };
        static const char* type() { return "mLMPsetscancfg"; }
        static const char* command_type() { return "sMN"; }
        comma::packed::uint32 scan_frequency; // 1/100Hz
        comma::packed::int16 sector; // 1 on LMS
        comma::packed::uint32 angular_resolution; // 1/10000 deg
        comma::packed::int32 start_angle; // 1/10000 deg, stupid sick: this setting doesn't get applied
        comma::packed::int32 stop_angle; // 1/10000 deg, stupid sick: this setting doesn't get applied
    };

    struct response : public comma::packed::packed_struct< response, 19 >
    {
        enum { type_field_size = 14 };
        static const char* type() { return "mLMPsetscancfg"; }
        static const char* command_type() { return "sAN"; }
        comma::packed::byte status_code;
        comma::packed::uint32 scan_frequency; // 1/100Hz
        comma::packed::int16 sector; // 1 on LMS
        comma::packed::uint32 angular_resolution; // 1/10000 deg
        comma::packed::int32 start_angle; // 1/10000 deg
        comma::packed::int32 stop_angle; // 1/10000 deg
    };
};

struct get_frequency_and_angular_resolution
{
    struct request : public comma::packed::packed_struct< request, 0 >
    {
        enum { type_field_size = 10 };
        static const char* type() { return "LMPscancfg"; }
        static const char* command_type() { return "sRN"; }
    };

    struct response : public comma::packed::packed_struct< response, 18 >
    {
        enum { type_field_size = 10 };
        static const char* type() { return "LMPscancfg"; }
        static const char* command_type() { return "sRA"; }
        comma::packed::uint32 scan_frequency; // 1/100 Hz
        comma::packed::int16 sector; // 1 on LMS
        comma::packed::uint32 angular_resolution; // 1/10000 deg
        comma::packed::int32 start_angle; // 1/10000 deg
        comma::packed::int32 stop_angle; // 1/10000 deg
    };
};

struct define_measurement_sectors
{
    struct request : public comma::packed::packed_struct< request, 18 >
    {
        enum { type_field_size = 14 };
        static const char* type() { return "mLMPsetscancfg"; }
        static const char* command_type() { return "sMN"; }
        comma::packed::uint32 scan_frequency;
        comma::packed::int16 number_of_sectors;
        comma::packed::uint32 angular_resolution; // resolution must be equal for all sectors
        comma::packed::int32 sector_start_angle;
        comma::packed::int32 sector_stop_angle;
    };

    struct response : public comma::packed::packed_struct< response, 19 >
    {
        enum { type_field_size = 14 };
        static const char* type() { return "mLMPsetscancfg"; }
        static const char* command_type() { return "sAN"; }
        comma::packed::byte error; // 0=none, 1=frequency, 2=resolution, 3=freq+res, 4=range, 5=general
        comma::packed::uint32 scan_frequency;
        comma::packed::int16 number_of_sectors;
        comma::packed::uint32 angular_resolution; // resolution must be equal for all sectors
        comma::packed::int32 sector_start_angle;
        comma::packed::int32 sector_stop_angle;
    };
};

struct set_scan_configuration // aka set interlace mode
{
    struct request : public comma::packed::packed_struct< request, 1 >
    {
        enum { type_field_size = 17 };
        static const char* type() { return "mCLsetscancfglist"; }
        static const char* command_type() { return "sMN"; }
        comma::packed::byte mode;
    };

    struct response : public comma::packed::packed_struct< response, 1 >
    {
        enum { type_field_size = 17 };
        static const char* type() { return "mCLsetscancfglist"; }
        static const char* command_type() { return "sAN"; }
        comma::packed::byte error; // 0=ok, 1=frequency, 2=resolution, 3=freq+res, 4=scan field, 5=error
    };
};

struct get_status
{
    struct request : public comma::packed::packed_struct< request, 0 >
    {
        enum { type_field_size = 11 };
        static const char* type() { return "LCMstate"; }
        static const char* command_type() { return "sRN"; }
    };

    struct response : public comma::packed::packed_struct< response, 1 >
    {
        enum { type_field_size = 11 };
        static const char* type() { return "LCMstate"; }
        static const char* command_type() { return "sRA"; }
        comma::packed::byte status; // 0=ok, 1=pollution warning, 2=pollution error, 3=fatal error
    };
};

struct configure_scan_data
{
    struct request : public comma::packed::packed_struct< request, 13 >
    {
        enum { type_field_size = 14 };
        static const char* type() { return "LMDscandatacfg"; }
        static const char* command_type() { return "sWN"; }
        comma::packed::byte data_channel; // 1=ch1, 2=ch2, 3=both
        comma::packed::byte zero1; // stupid sick: this is meant to be part of the data_channel but it's always zero?
        comma::packed::byte include_remission; // todo: bool?
        comma::packed::byte resolution; // 0=8-bit, 1=16-bit
        comma::packed::byte unit; // stupid sick: unit of remission data, but it's always 0
        comma::packed::byte encoder; // 0=none, 1=ch1
        comma::packed::byte zero2; // stupid sick: second byte for encoder setting, but it's always zero
        comma::packed::byte include_position;
        comma::packed::byte include_device_name;
        comma::packed::byte include_comment;
        comma::packed::byte include_time;
        comma::packed::uint16 output_rate; // skip every Nth scan
    };

    struct response : public comma::packed::packed_struct< response, 0 >
    {
        enum { type_field_size = 14 };
        static const char* type() { return "LMDscandatacfg"; }
        static const char* command_type() { return "sWA"; }
        // stupid sick: apparently the response has no data
        // stupid sick: but apparently the response DOES have a space between the 'type' and the crc!!
    };
};

struct set_measurement_angle
{
    struct request : public comma::packed::packed_struct< request, 14 >
    {
        enum { type_field_size = 11 };
        static const char* type() { return "LMPoutputRange"; }
        static const char* command_type() { return "sWN"; }
        comma::packed::int16 status_code; // stupid sick: set to 1
        comma::packed::uint32 angular_resolution; // stupid sick: cannot be changed here
        comma::packed::int32 start_angle; // 1/10000 deg
        comma::packed::int32 stop_angle; // 1/10000 deg
    };

    struct response : public comma::packed::packed_struct< response, 0 >
    {
        enum { type_field_size = 11 };
        static const char* type() { return "LMPoutputRange"; }
        static const char* command_type() { return "sWA"; }
        // stupid sick: apparently the response has no data
        // stupid sick: but apparently the response DOES have a space between the 'type' and the crc!!
    };
};

struct get_measurement_angle
{
    struct request : public comma::packed::packed_struct< request, 0 >
    {
        enum { type_field_size = 11 };
        static const char* type() { return "LMPoutputRange"; }
        static const char* command_type() { return "sRN"; }
    };

    struct response : public comma::packed::packed_struct< response, 14 >
    {
        enum { type_field_size = 11 };
        static const char* type() { return "LMPoutputRange"; }
        static const char* command_type() { return "sRA"; }
        // stupid sick: this is undocumented, but hopefully it's right (assuming same as set_measurement_angle)
        comma::packed::int16 status_code;
        comma::packed::uint32 angular_resolution;
        comma::packed::int32 start_angle;
        comma::packed::int32 stop_angle;
    };
};

struct poll_single_scan
{
    struct request : public comma::packed::packed_struct< request, 0 >
    {
        enum { type_field_size = 11 };
        static const char* type() { return "LMDscandata"; }
        static const char* command_type() { return "sRN"; }
    };

    struct response : public comma::packed::packed_struct< response, 0 >
    {
        enum { type_field_size = 11 };
        static const char* type() { return "LMDscandata"; }
        static const char* command_type() { return "sRA"; } // documentation says sRS but maybe typo?
        // todo, defined below
    };
};

struct scan
{
    struct request : public comma::packed::packed_struct< request, 1 >
    {
        enum { type_field_size = 11 };
        static const char* type() { return "LMDscandata"; }
        static const char* command_type() { return "sEN"; }
        comma::packed::byte state; // 0=stop, 1=start
    };

    struct response : public comma::packed::packed_struct< response, 1 >
    {
        enum { type_field_size = 11 };
        static const char* type() { return "LMDscandata"; }
        static const char* command_type() { return "sEA"; }
        comma::packed::byte status; // 0=stopped, 1=started
    };
};

struct scan_data
{
    struct request; // no request for this

    // todo: actual size is variable
    struct response : public comma::packed::packed_struct< response, 0 >
    {
        enum { type_field_size = 11 };
        static const char* type() { return "LMDscandata"; }
        static const char* command_type() { return "sRA"; } // can also be sSN? typo?
        comma::packed::uint16 version;
        // device (8)
        comma::packed::uint16 device_number; // user-defined in SOPAS
        comma::packed::uint32 serial_number;
        comma::packed::byte zero1; // stupid sick: apparently half of device status, but always zero
        comma::packed::byte status; // 0=ok, 1=error, 2=pollution warning, 3=pollution error
        // status info (18)
        comma::packed::uint16 telegram_counter;
        comma::packed::uint16 scan_counter;
        comma::packed::uint32 time_since_boot;
        comma::packed::uint32 time_of_transmission;
        comma::packed::byte zero2; // stupid sick: unused status byte
        comma::packed::byte digital_input_state; // 0=all low, 3=all high
        comma::packed::byte zero3; // stupid sick: unused status byte
        comma::packed::byte digital_output_state; // 0=all low, 7=all high
        comma::packed::uint16 reserved;
        // frequency (8)
        comma::packed::uint32 scan_frequency;
        comma::packed::uint32 shot_frequency;
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
        // comma::packed::int32 event_angle; // encoder angle at time of event

    };
};

struct set_date_time
{
    struct request : public comma::packed::packed_struct< request, 11 >
    {
        enum { type_field_size = 14 };
        static const char* type() { return "LSPsetdatetime"; }
        static const char* command_type() { return "sMN"; }
        comma::packed::uint16 year;
        comma::packed::uint8 month;
        comma::packed::uint8 day;
        comma::packed::uint8 hour;
        comma::packed::uint8 minute;
        comma::packed::uint8 second;
        comma::packed::uint32 microseconds;
    };

    struct response : public comma::packed::packed_struct< response, 1 >
    {
        enum { type_field_size = 14 };
        static const char* type() { return "LSPsetdatetime"; }
        static const char* command_type() { return "sAN"; }
        comma::packed::byte status; // 0=fail, 1=success
    };
};

struct get_date_time
{
    struct request : public comma::packed::packed_struct< request, 0 >
    {
        enum { type_field_size = 5 };
        static const char* type() { return "STlms"; }
        static const char* command_type() { return "sRN"; }
    };

    struct response : public comma::packed::packed_struct< response, 1 >
    {
        enum { type_field_size = 5 };
        static const char* type() { return "STlms"; }
        static const char* command_type() { return "sRA"; }
        comma::packed::uint16 status; // 0=undefined, 1=initialisation, 2=configuration, 3=lower case, 4=rotating, 5=in prep, 6=ready, 7=measurement active
        comma::packed::uint8 op_temp_range; // no documentation
        comma::packed::uint16 unnamed_field; // no documentation
        comma::packed::uint16 time_hours; // HH HH (stupid sick: I think this is actually ascii characters)
        comma::packed::byte time_separator1; // stupid sick: this is a ':' character
        comma::packed::uint16 time_minutes; // MM MM
        comma::packed::byte time_separator2; // ':'
        comma::packed::uint16 time_seconds; // SS SS
        comma::packed::uint16 unnamed_field2; // ?
        comma::packed::uint16 date_day; // DD DD
        comma::packed::byte separator1; // '.'
        comma::packed::uint16 date_month; // MM MM
        comma::packed::byte separator2; // '.'
        comma::packed::uint32 year; // JJ JJ JJ JJ
        comma::packed::uint16 led1_state; // 0=inactive, 1=active
        comma::packed::uint16 led2_state; // 0=inactive, 1=active
        comma::packed::uint16 led3_state; // 0=inactive, 1=active
    };
};

struct get_device_time_counter
{
    struct request : public comma::packed::packed_struct< request, 0 >
    {
        enum { type_field_size = 10 }; // stupid sick: documentation says 5
        static const char* type() { return "DeviceTime"; }
        static const char* command_type() { return "sRN"; }
    };

    struct response : public comma::packed::packed_struct< response, 4 >
    {
        enum { type_field_size = 10 };
        static const char* type() { return "DeviceTime"; }
        static const char* command_type() { return "sRA"; }
        comma::packed::uint32 time; // current value of the 32-bit counter, which is probably in milliseconds but the documentation is self-conflicting (maybe microseconds)
    };
};

struct save_settings_to_eeprom
{
    struct request : public comma::packed::packed_struct< request, 0 >
    {
        enum { type_field_size = 11 };
        static const char* type() { return "mEEwriteall"; }
        static const char* command_type() { return "sMN"; }
    };

    struct response : public comma::packed::packed_struct< response, 1 >
    {
        enum { type_field_size = 11 };
        static const char* type() { return "mEEwriteall"; }
        static const char* command_type() { return "sAN"; }
        comma::packed::byte status; // 0=error, 1=success
    };
};

struct run // also known as logout in some documentation
{
    struct request : public comma::packed::packed_struct< request, 0 >
    {
        enum { type_field_size = 3 };
        static const char* type() { return "Run"; }
        static const char* command_type() { return "sMN"; }
    };

    struct response : public comma::packed::packed_struct< response, 1 >
    {
        enum { type_field_size = 3 };
        static const char* type() { return "Run"; }
        static const char* command_type() { return "sAN"; }
        comma::packed::byte status; // 1=success, 0=error
    };
};

struct set_particle_filter // also known as logout in some documentation
{
    struct request : public comma::packed::packed_struct< request, 3 >
    {
        enum { type_field_size = 11 };
        static const char* type() { return "LFPparticle"; }
        static const char* command_type() { return "sWN"; }
        comma::packed::byte state; // 0=inactive, 1=active
        comma::packed::uint16 threshold; // stupid sick: documentation is very unclear but has a red highlighted note here... i *THINK* it has to be set to 500 always or the world will end.
    };

    struct response : public comma::packed::packed_struct< response, 0 >
    {
        enum { type_field_size = 11 };
        static const char* type() { return "LFPparticle"; }
        static const char* command_type() { return "sWA"; }
        // todo: documentation includes a space after this command
        // todo: documentation also looks like a copy-paste error so could be entirely wrong.
    };
};

struct set_mean_filter
{
    struct request : public comma::packed::packed_struct< request, 4 >
    {
        enum { type_field_size = 13 };
        static const char* type() { return "LFPmeanfilter"; }
        static const char* command_type() { return "sWN"; }
        comma::packed::byte state; // 0=inactive, 1=active
        comma::packed::uint16 number_of_scans; // 2-100
        comma::packed::byte zero; // stupid sick: an unnamed zero value...
    };

    struct response : public comma::packed::packed_struct< response, 0 >
    {
        enum { type_field_size = 13 };
        static const char* type() { return "LFPmeanfilter"; }
        static const char* command_type() { return "sWA"; }
        // todo: documentation includes a space after this command
    };
};

struct set_n_to_one_filter
{
    struct request : public comma::packed::packed_struct< request, 1 >
    {
        enum { type_field_size = 13 };
        static const char* type() { return "LFPnto1filter"; }
        static const char* command_type() { return "sWN"; }
        comma::packed::byte state; // 0=inactive, 1=active
    };

    struct response : public comma::packed::packed_struct< response, 0 >
    {
        enum { type_field_size = 13 };
        static const char* type() { return "LFPnto1filter"; }
        static const char* command_type() { return "sWA"; }
        // todo: documentation includes a space after this command
    };
};

struct set_echo_filter
{
    struct request : public comma::packed::packed_struct< request, 1 >
    {
        enum { type_field_size = 12 };
        static const char* type() { return "FREchoFilter"; }
        static const char* command_type() { return "sWN"; }
        comma::packed::byte state; // 0=first echo, 1=all echos, 2=last echo
        // todo: Not available in binary in V1.10 firmware
    };

    struct response : public comma::packed::packed_struct< response, 0 >
    {
        enum { type_field_size = 12 };
        static const char* type() { return "FREchoFilter"; }
        static const char* command_type() { return "sWA"; }
        // todo: Not available in binary in V1.10 firmware
    };
};

struct set_fog_filter
{
    struct request : public comma::packed::packed_struct< request, 1 >
    {
        enum { type_field_size = 10 };
        static const char* type() { return "MSsuppmode"; }
        static const char* command_type() { return "sWN"; }
        comma::packed::byte state; // 0=glitch, 1=fog
    };

    struct response : public comma::packed::packed_struct< response, 0 >
    {
        enum { type_field_size = 10 };
        static const char* type() { return "MSsuppmode"; }
        static const char* command_type() { return "sWA"; }
    };
    // todo: documentation includes a space after this command
};

struct set_nearfield_filter
{
    struct request : public comma::packed::packed_struct< request, 1 >
    {
        enum { type_field_size = 15 };
        static const char* type() { return "CLNFDigFilterEn"; }
        static const char* command_type() { return "sWN"; }
        comma::packed::byte state; // 0=off, 1=on
    };

    struct response : public comma::packed::packed_struct< response, 0 >
    {
        enum { type_field_size = 15 };
        static const char* type() { return "CLNFDigFilterEn"; }
        static const char* command_type() { return "sWA"; }
    };
    // todo: documentation does NOT include a space after this command (inconsistent compared to other dataless binary responses)
};

struct set_nearfield_filter_gating
{
    struct request : public comma::packed::packed_struct< request, 1 >
    {
        enum { type_field_size = 10 };
        static const char* type() { return "CLHWGating"; }
        static const char* command_type() { return "sWN"; }
        comma::packed::byte state; // 0=off, 3=2m, 6=5m
    };

    struct response : public comma::packed::packed_struct< response, 0 >
    {
        enum { type_field_size = 10 };
        static const char* type() { return "CLHWGating"; }
        static const char* command_type() { return "sWA"; }
    };
    // todo: documentation does NOT include a space after this command (inconsistent compared to other dataless binary responses)
};

struct set_nearfield_filter_sector
{
    struct request : public comma::packed::packed_struct< request, 4 >
    {
        enum { type_field_size = 16 };
        static const char* type() { return "CLHWFilterSectEn"; }
        static const char* command_type() { return "sWN"; }
        comma::packed::byte state_sector1; // 0=inactive, 1=active
        comma::packed::byte state_sector2;
        comma::packed::byte state_sector3;
        comma::packed::byte state_sector4;
    };

    struct response : public comma::packed::packed_struct< response, 0 >
    {
        enum { type_field_size = 16 };
        static const char* type() { return "CLHWFilterSectEn"; }
        static const char* command_type() { return "sWA"; }
    };
    // todo: documentation does NOT include a space after this command (inconsistent compared to other dataless binary responses)
};

struct set_encoder_source
{
    struct request : public comma::packed::packed_struct< request, 1 >
    {
        enum { type_field_size = 6 };
        static const char* type() { return "LICsrc"; }
        static const char* command_type() { return "sWN"; }
        comma::packed::byte state; // 0=fixed speed, 1=encoder
    };

    struct response : public comma::packed::packed_struct< response, 0 >
    {
        enum { type_field_size = 6 };
        static const char* type() { return "LICsrc"; }
        static const char* command_type() { return "sWA"; }
    };
    // todo: documentation does NOT include a space after this command (inconsistent compared to other dataless binary responses)
};

struct set_encoder_mode
{
    struct request : public comma::packed::packed_struct< request, 1 >
    {
        enum { type_field_size = 9 };
        static const char* type() { return "LICencset"; }
        static const char* command_type() { return "sWN"; }
        comma::packed::byte mode; // 0=off, 1=single increment, 2=direction recognition (phase), 3=direction recognition (level)
    };

    struct response : public comma::packed::packed_struct< response, 0 >
    {
        enum { type_field_size = 9 };
        static const char* type() { return "LICencset"; }
        static const char* command_type() { return "sWA"; }
    };
    // todo: documentation does NOT include a space after this command (inconsistent compared to other dataless binary responses)
};

struct set_encoder_resolution
{
    struct request : public comma::packed::packed_struct< request, 4 >
    {
        enum { type_field_size = 9 };
        static const char* type() { return "LICencres"; }
        static const char* command_type() { return "sWN"; }
        comma::packed::uint32 resolution; // stupid sick: no data type specified so be warned... range is 0.001 to 2000
    };

    struct response : public comma::packed::packed_struct< response, 0 >
    {
        enum { type_field_size = 9 };
        static const char* type() { return "LICencres"; }
        static const char* command_type() { return "sWA"; }
    };
    // todo: maybe not available in binary as no binary example is given?
};

struct set_encoder_fixed_speed
{
    struct request : public comma::packed::packed_struct< request, 4 >
    {
        enum { type_field_size = 8 };
        static const char* type() { return "LICFixVel"; }
        static const char* command_type() { return "sWN"; }
        comma::packed::uint32 speed; // stupid sick: no data type specified so be warned... range is 0.001 to 10
    };

    struct response : public comma::packed::packed_struct< response, 0 >
    {
        enum { type_field_size = 8 };
        static const char* type() { return "LICFixVel"; }
        static const char* command_type() { return "sWA"; }
    };
    // todo: maybe not available in binary as no binary example is given?
};

struct get_encoder_speed_threshold
{
    struct request : public comma::packed::packed_struct< request, 0 >
    {
        enum { type_field_size = 7 };
        static const char* type() { return "LICSpTh"; }
        static const char* command_type() { return "sRN"; }
    };

    struct response : public comma::packed::packed_struct< response, 1 >
    {
        enum { type_field_size = 7 };
        static const char* type() { return "LICSpTh"; }
        static const char* command_type() { return "sRA"; }
        comma::packed::byte speed; // binary example shows '05' hex but field is not present in documentation
    };
};

struct get_encoder_speed
{
    struct request : public comma::packed::packed_struct< request, 0 >
    {
        enum { type_field_size = 7 };
        static const char* type() { return "LICencsp"; }
        static const char* command_type() { return "sRN"; }
    };

    struct response : public comma::packed::packed_struct< response, 1 >
    {
        enum { type_field_size = 7 };
        static const char* type() { return "LICencsp"; }
        static const char* command_type() { return "sRA"; }
        comma::packed::uint32 speed;
    };
    // todo: check this, the documentation puts the speed in the request and doesn't mention any type
    // ... but that makes no sense... right?
};

struct get_output_states
{
    struct request : public comma::packed::packed_struct< request, 0 >
    {
        enum { type_field_size = 14 };
        static const char* type() { return "LIDoutputstate"; }
        static const char* command_type() { return "sRN"; }
    };

    struct response : public comma::packed::packed_struct< response, 0 >
    {
        enum { type_field_size = 14 };
        static const char* type() { return "LIDoutputstate"; }
        static const char* command_type() { return "sRA"; }
        comma::packed::uint16 status_code;
        comma::packed::uint32 unnamed_value; // stupid sick
        // 8-bit state values are 0=low, 1=high, 2=tristate (undefined)
        comma::packed::byte state_output1;
        comma::packed::uint32 count_output1;
        comma::packed::byte state_output2;
        comma::packed::uint32 count_output2;
        comma::packed::byte state_output3;
        comma::packed::uint32 count_output3;
        // todo: these are only present for LMS5xx not LMS1xx
        // comma::packed::byte state_output4;
        // comma::packed::uint32 count_output4;
        // comma::packed::byte state_output5;
        // comma::packed::uint32 count_output5;
        // comma::packed::byte state_output6;
        // comma::packed::uint32 count_output6;
        comma::packed::byte external_state_output1;
        comma::packed::uint32 external_state_count1;
        comma::packed::byte external_state_output2;
        comma::packed::uint32 external_state_count2;
        comma::packed::byte external_state_output3;
        comma::packed::uint32 external_state_count3;
        comma::packed::byte external_state_output4;
        comma::packed::uint32 external_state_count4;
        comma::packed::byte external_state_output5;
        comma::packed::uint32 external_state_count5;
        comma::packed::byte external_state_output6;
        comma::packed::uint32 external_state_count6;
        comma::packed::byte external_state_output7;
        comma::packed::uint32 external_state_count7;
        comma::packed::byte external_state_output8;
        comma::packed::uint32 external_state_count8;
        comma::packed::uint16 reserved;
    };
    // todo: variable size with laser model
    // LMS5xx has all outputs
    // LMS1xx has 3 outputs
    // NAV310 has 1 output...
};

struct set_output_states
{
    struct request : public comma::packed::packed_struct< request, 2 >
    {
        enum { type_field_size = 12 };
        static const char* type() { return "mDOSetOutput"; }
        static const char* command_type() { return "sMN"; }
        comma::packed::uint8 output_number; // 1-3
        comma::packed::uint8 state; // 0=inactive, 1=active
    };

    struct response : public comma::packed::packed_struct< response, 1 >
    {
        enum { type_field_size = 12 };
        static const char* type() { return "mDOSetOutput"; }
        static const char* command_type() { return "sAN"; }
        comma::packed::byte state; // 1=success, 0=error
    };
};

struct set_output_6_function
{
    struct request : public comma::packed::packed_struct< request, 1 >
    {
        enum { type_field_size = 6 };
        static const char* type() { return "DO6Fnc"; }
        static const char* command_type() { return "sWN"; }
        comma::packed::byte function; // 0=no function, 1=command, 2=dev ready, 3=application, 4=app/dev rdy, 5=dev rdy/poll, 6=pollution, 7=zero-index (master sync)
    };

    struct response : public comma::packed::packed_struct< response, 0 >
    {
        enum { type_field_size = 6 };
        static const char* type() { return "DO6Fnc"; }
        static const char* command_type() { return "sWA"; }
    };
    // todo: not available in binary in v1.10
};

struct set_output_3_function
{
    struct request : public comma::packed::packed_struct< request, 1 >
    {
        enum { type_field_size = 6 };
        static const char* type() { return "DO3Fnc"; }
        static const char* command_type() { return "sWN"; }
        comma::packed::byte function; // 0=no function, 1=command, 2=dev ready, 3=application, 4=app/dev rdy, 5=dev rdy/poll, 6=pollution, 7=zero-index (master sync)
    };

    struct response : public comma::packed::packed_struct< response, 0 >
    {
        enum { type_field_size = 6 };
        static const char* type() { return "DO3Fnc"; }
        static const char* command_type() { return "sWA"; }
    };
    // todo: not available in binary in v1.10
};

struct set_output_1_function
{
    struct request : public comma::packed::packed_struct< request, 1 >
    {
        enum { type_field_size = 6 };
        static const char* type() { return "DO1Fnc"; }
        static const char* command_type() { return "sWN"; }
        comma::packed::byte function; // 0=no function, 1=command, 2=dev ready, 3=application/dev rdy, 4=sync pulse, 5=sync index (default)
    };

    struct response : public comma::packed::packed_struct< response, 0 >
    {
        enum { type_field_size = 6 };
        static const char* type() { return "DO1Fnc"; }
        static const char* command_type() { return "sWA"; }
    };
};

struct set_output_1_logic
{
    struct request : public comma::packed::packed_struct< request, 1 >
    {
        enum { type_field_size = 8 };
        static const char* type() { return "DO1Logic"; }
        static const char* command_type() { return "sWN"; }
        comma::packed::byte logic; // 0=active high (default), 1=active low
    };

    struct response : public comma::packed::packed_struct< response, 0 >
    {
        enum { type_field_size = 8 };
        static const char* type() { return "DO1Logic"; }
        static const char* command_type() { return "sWA"; }
    };
};

struct set_output_2_function
{
    struct request : public comma::packed::packed_struct< request, 1 >
    {
        enum { type_field_size = 6 };
        static const char* type() { return "DO2Fnc"; }
        static const char* command_type() { return "sWN"; }
        comma::packed::byte function; // 0=no function, 1=command, 2=device ready, 3=application/dev ready
    };

    struct response : public comma::packed::packed_struct< response, 0 >
    {
        enum { type_field_size = 6 };
        static const char* type() { return "DO2Fnc"; }
        static const char* command_type() { return "sWA"; }
    };
};

struct set_output_2_logic
{
    struct request : public comma::packed::packed_struct< request, 1 >
    {
        enum { type_field_size = 8 };
        static const char* type() { return "DO2Logic"; }
        static const char* command_type() { return "sWN"; }
        comma::packed::byte logic; // 0=active high (default), 1=active low
    };

    struct response : public comma::packed::packed_struct< response, 0 >
    {
        enum { type_field_size = 8 };
        static const char* type() { return "DO2Logic"; }
        static const char* command_type() { return "sWA"; }
    };
};

struct set_input_4_function
{
    struct request : public comma::packed::packed_struct< request, 1 >
    {
        enum { type_field_size = 10 };
        static const char* type() { return "DO3And4Fnc"; }
        static const char* command_type() { return "sWN"; }
        comma::packed::byte function; // 0=no function, 1=encoder, 2=slave sync, 3=digital input
    };

    struct response : public comma::packed::packed_struct< response, 0 >
    {
        enum { type_field_size = 10 };
        static const char* type() { return "DO3And4Fnc"; }
        static const char* command_type() { return "sWA"; }
    };
    // binary Not available with firmware V1.10
};

struct reset_output_counter
{
    struct request : public comma::packed::packed_struct< request, 0 >
    {
        enum { type_field_size = 13 };
        static const char* type() { return "LIDrstoutpcnt"; }
        static const char* command_type() { return "sMN"; }
    };

    struct response : public comma::packed::packed_struct< response, 1 >
    {
        enum { type_field_size = 13 };
        static const char* type() { return "LIDrstoutpcnt"; }
        static const char* command_type() { return "sAN"; }
        comma::packed::byte state; // 0=success
    };
};

struct get_device_identity
{
    struct request : public comma::packed::packed_struct< request, 0 >
    {
        enum { type_field_size = 11 };
        static const char* type() { return "DeviceIdent"; }
        static const char* command_type() { return "sRN"; } // apparently also sRI? probably not really stupid sick
    };

    struct response : public comma::packed::packed_struct< response, 1 >
    {
        enum { type_field_size = 11 };
        static const char* type() { return "DeviceIdent"; }
        static const char* command_type() { return "sRA"; }
        // string length defined by packet size
        //comma::packed::string< N > state; // 0=success
    };
};

struct get_device_state
{
    struct request : public comma::packed::packed_struct< request, 0 >
    {
        enum { type_field_size = 13 };
        static const char* type() { return "SCdevicestate"; }
        static const char* command_type() { return "sRN"; }
    };

    struct response : public comma::packed::packed_struct< response, 1 >
    {
        enum { type_field_size = 13 };
        static const char* type() { return "SCdevicestate"; }
        static const char* command_type() { return "sRA"; }
        comma::packed::byte state; // 0=busy, 1=ready, 2=error
    };
};

struct set_device_name
{
    struct request : public comma::packed::packed_struct< request, 18 >
    {
        enum { type_field_size = 12 };
        static const char* type() { return "LocationName"; }
        static const char* command_type() { return "sWN"; }
        comma::packed::uint16 name_length_in_ascii_hex; // note, this value is written in ASCII as HEX even though it's a "binary" protocol - fucking stupid sick
        comma::packed::string< 16 > name; // example shows shorter string based on ascii hex length - stupid sick
    };

    struct response : public comma::packed::packed_struct< response, 18 >
    {
        enum { type_field_size = 12 };
        static const char* type() { return "LocationName"; }
        static const char* command_type() { return "sWA"; }
        comma::packed::uint16 name_length_in_ascii_hex; // note, this value is written in ASCII as HEX even though it's a "binary" protocol - fucking stupid sick
        comma::packed::string< 16 > name; // example shows shorter string based on ascii hex length - stupid sick
        // todo: the example doesn't include the length, so who knows if it's really there
    };
};

struct get_device_name
{
    struct request : public comma::packed::packed_struct< request, 0 >
    {
        enum { type_field_size = 12 };
        static const char* type() { return "LocationName"; }
        static const char* command_type() { return "sRN"; }
    };

    struct response : public comma::packed::packed_struct< response, 18 >
    {
        enum { type_field_size = 12 };
        static const char* type() { return "LocationName"; }
        static const char* command_type() { return "sRA"; }
        comma::packed::uint16 name_length_in_ascii_hex; // note, this value is written in ASCII as HEX even though it's a "binary" protocol - fucking stupid sick
        comma::packed::string< 16 > name; // example shows shorter string based on ascii hex length - stupid sick
    };
};

struct get_operating_hours
{
    struct request : public comma::packed::packed_struct< request, 0 >
    {
        enum { type_field_size = 6 };
        static const char* type() { return "ODoprh"; }
        static const char* command_type() { return "sRN"; }
    };

    struct response : public comma::packed::packed_struct< response, 4 >
    {
        enum { type_field_size = 6 };
        static const char* type() { return "ODoprh"; }
        static const char* command_type() { return "sRA"; }
        comma::packed::uint32 operating_hours; // 1/10h
    };
};

struct get_power_on_counter
{
    struct request : public comma::packed::packed_struct< request, 0 >
    {
        enum { type_field_size = 6 };
        static const char* type() { return "ODpwrc"; }
        static const char* command_type() { return "sRN"; }
    };

    struct response : public comma::packed::packed_struct< response, 4 >
    {
        enum { type_field_size = 6 };
        static const char* type() { return "ODpwrc"; }
        static const char* command_type() { return "sRA"; }
        comma::packed::uint32 counter; // units not documented...
    };
};

struct set_ip_address
{
    struct request : public comma::packed::packed_struct< request, 4 >
    {
        enum { type_field_size = 8 };
        static const char* type() { return "EIIpAddr"; }
        static const char* command_type() { return "sWN"; }
        comma::packed::uint32 ip_address; // one byte per segment, surprisingly sensible
    };

    struct response : public comma::packed::packed_struct< response, 0 >
    {
        enum { type_field_size = 8 };
        static const char* type() { return "EIIpAddr"; }
        static const char* command_type() { return "sWA"; }
        // todo: note the random space at the end
    };
};

struct set_ip_gateway
{
    struct request : public comma::packed::packed_struct< request, 4 >
    {
        enum { type_field_size = 6 };
        static const char* type() { return "EIgate"; }
        static const char* command_type() { return "sWN"; }
        comma::packed::uint32 gateway; // one byte per segment, surprisingly sensible
    };

    struct response : public comma::packed::packed_struct< response, 0 >
    {
        enum { type_field_size = 6 };
        static const char* type() { return "EIgate"; }
        static const char* command_type() { return "sWA"; }
        // todo: no random space at the end
    };
};

struct set_ip_netmask
{
    struct request : public comma::packed::packed_struct< request, 4 >
    {
        enum { type_field_size = 6 };
        static const char* type() { return "EImask"; }
        static const char* command_type() { return "sWN"; }
        comma::packed::uint32 netmask; // one byte per segment, surprisingly sensible
    };

    struct response : public comma::packed::packed_struct< response, 0 >
    {
        enum { type_field_size = 6 };
        static const char* type() { return "EImask"; }
        static const char* command_type() { return "sWA"; }
        // todo: no random space at the end
    };
};

struct get_angle_compensation_sine
{
    // NAV310 only
    struct request : public comma::packed::packed_struct< request, 0 >
    {
        enum { type_field_size = 14 };
        static const char* type() { return "MCAngleCompSin"; }
        static const char* command_type() { return "sRN"; }
    };

    struct response : public comma::packed::packed_struct< response, 8 >
    {
        enum { type_field_size = 14 };
        static const char* type() { return "MCAngleCompSin"; }
        static const char* command_type() { return "sRA"; }
        comma::packed::int16 amplitude; // 1/10000 deg, -10,000 to +10,000
        comma::packed::int32 phase; // 1/10000 deg, -3,600,000 to +3,600,000
        comma::packed::int16 offset; // 1/1000 deg (different! maybe!?), -10,000 to +10,000
        // todo: warning the example binary has space separators and doens't respect the above binary types
    };
};

struct reset_factory_defaults
{
    struct request : public comma::packed::packed_struct< request, 0 >
    {
        enum { type_field_size = 13 };
        static const char* type() { return "mSCloadfacdef"; }
        static const char* command_type() { return "sMN"; }
    };

    struct response : public comma::packed::packed_struct< response, 0 >
    {
        enum { type_field_size = 13 };
        static const char* type() { return "mSCloadfacdef"; }
        static const char* command_type() { return "sAN"; }
    };
    // "not possible" in binary
};

struct reset_application_factory_defaults
{
    struct request : public comma::packed::packed_struct< request, 0 >
    {
        enum { type_field_size = 13 };
        static const char* type() { return "mSCloadappdef"; }
        static const char* command_type() { return "sMN"; }
    };

    struct response : public comma::packed::packed_struct< response, 0 >
    {
        enum { type_field_size = 13 };
        static const char* type() { return "mSCloadappdef"; }
        static const char* command_type() { return "sAN"; }
    };
    // "not possible" in binary
};

struct reboot
{
    struct request : public comma::packed::packed_struct< request, 0 >
    {
        enum { type_field_size = 9 };
        static const char* type() { return "mSCreboot"; }
        static const char* command_type() { return "sMN"; }
    };

    struct response : public comma::packed::packed_struct< response, 0 >
    {
        enum { type_field_size = 9 };
        static const char* type() { return "mSCreboot"; }
        static const char* command_type() { return "sAN"; }
    };
};

struct set_contamination_values
{
    struct request : public comma::packed::packed_struct< request, 13 >
    {
        enum { type_field_size = 6 };
        static const char* type() { return "LCMcfg"; }
        static const char* command_type() { return "sWN"; }
        comma::packed::uint8 strategy; // 0=inactive, 1=high available, 2=available, 3=sensitive, 4=semi-sensitive
        comma::packed::uint32 response_time; // 1-60
        comma::packed::uint32 threshold_warning; // 0-100
        comma::packed::uint32 threshold_error; // 0-100
    };

    struct response : public comma::packed::packed_struct< response, 0 >
    {
        enum { type_field_size = 6 };
        static const char* type() { return "LCMcfg"; }
        static const char* command_type() { return "sWA"; }
        // note the random space at end
    };
};

struct get_contamination_values
{
    struct request : public comma::packed::packed_struct< request, 0 >
    {
        enum { type_field_size = 6 };
        static const char* type() { return "LCMcfg"; }
        static const char* command_type() { return "sRN"; }
    };

    struct response : public comma::packed::packed_struct< response, 7 >
    {
        enum { type_field_size = 6 };
        static const char* type() { return "LCMcfg"; }
        static const char* command_type() { return "sRA"; }
        comma::packed::uint8 strategy; // 0=inactive, 1=high available, 2=available, 3=sensitive, 4=semi-sensitive
        comma::packed::uint16 response_time; // 1-60
        comma::packed::uint16 threshold_warning; // 0-100
        comma::packed::uint16 threshold_error; // 0-100
        // stupid sick: yes, even though you set them as uint32, they come back as uint16
    };
};

struct set_synchronisation_phase
{
    struct request : public comma::packed::packed_struct< request, 13 >
    {
        enum { type_field_size = 7 };
        static const char* type() { return "SYPhase"; }
        static const char* command_type() { return "sWN"; }
        comma::packed::uint8 phase; // units probably degrees, field entirely unspecified in documentation - stupid sick
    };

    struct response : public comma::packed::packed_struct< response, 0 >
    {
        enum { type_field_size = 7 };
        static const char* type() { return "SYPhase"; }
        static const char* command_type() { return "sWA"; }
    };
    // binary not available in firmware v1.10
};

struct set_front_panel
{
    struct request : public comma::packed::packed_struct< request, 4 >
    {
        enum { type_field_size = 8 };
        static const char* type() { return "LMLfpFcn"; }
        static const char* command_type() { return "sWN"; }
        comma::packed::uint8 reserved;
        comma::packed::uint8 led_function_q1q2; // 0=none, 1=app, 2=cmd
        comma::packed::uint8 led_function_okstop; // 0=none, 1=app, 2=cmd
        comma::packed::uint8 display_function; // 0=app, 1=cmd
    };

    struct response : public comma::packed::packed_struct< response, 0 >
    {
        enum { type_field_size = 8 };
        static const char* type() { return "LMLfpFcn"; }
        static const char* command_type() { return "sWA"; }
        // note the random space at end maybe
    };
};

struct set_function_led_1
{
    struct request : public comma::packed::packed_struct< request, 1 >
    {
        enum { type_field_size = 11 };
        static const char* type() { return "HMIfpFcn_Y1"; }
        static const char* command_type() { return "sWN"; }
        comma::packed::uint8 mode; // 0=no function, 1=app, 2=cmd
    };

    struct response : public comma::packed::packed_struct< response, 0 >
    {
        enum { type_field_size = 11 };
        static const char* type() { return "HMIfpFcn_Y1"; }
        static const char* command_type() { return "sWA"; }
    };
};

struct set_function_led_2
{
    struct request : public comma::packed::packed_struct< request, 1 >
    {
        enum { type_field_size = 11 };
        static const char* type() { return "HMIfpFcn_Y2"; }
        static const char* command_type() { return "sWN"; }
        comma::packed::uint8 mode; // 0=no function, 1=app, 2=cmd
    };

    struct response : public comma::packed::packed_struct< response, 0 >
    {
        enum { type_field_size = 11 };
        static const char* type() { return "HMIfpFcn_Y2"; }
        static const char* command_type() { return "sWA"; }
    };
    // to be honest the documentation on this one is so bad I'm not sure if it enables LED1 or configures LED2
};

struct set_led_state
{
    struct request : public comma::packed::packed_struct< request, 2 >
    {
        enum { type_field_size = 10 };
        static const char* type() { return "mHMISetLed"; }
        static const char* command_type() { return "sMN"; }
        comma::packed::uint8 led_id; // 3=LED1, 4=LED2 of course
        comma::packed::uint8 state; // 0=off, 1=on
    };

    struct response : public comma::packed::packed_struct< response, 1 >
    {
        enum { type_field_size = 10 };
        static const char* type() { return "mHMISetLed"; }
        static const char* command_type() { return "sAN"; }
        comma::packed::uint8 result; // 1=success, 0=fail
    };
};

struct standby
{
    struct request : public comma::packed::packed_struct< request, 0 >
    {
        enum { type_field_size = 10 };
        static const char* type() { return "LMCstandby"; }
        static const char* command_type() { return "sMN"; }
    };

    struct response : public comma::packed::packed_struct< response, 1 >
    {
        enum { type_field_size = 10 };
        static const char* type() { return "LMCstandby"; }
        static const char* command_type() { return "sAN"; }
        comma::packed::uint8 status_code; // 0=no error
    };
};

struct start_measurement
{
    struct request : public comma::packed::packed_struct< request, 0 >
    {
        enum { type_field_size = 12 };
        static const char* type() { return "LMCstartmeas"; }
        static const char* command_type() { return "sMN"; }
    };

    struct response : public comma::packed::packed_struct< response, 1 >
    {
        enum { type_field_size = 12 };
        static const char* type() { return "LMCstartmeas"; }
        static const char* command_type() { return "sAN"; }
        comma::packed::uint8 status_code; // 0=no error, 1=not allowed
    };
};

struct stop_measurement
{
    struct request : public comma::packed::packed_struct< request, 0 >
    {
        enum { type_field_size = 11 };
        static const char* type() { return "LMCstopmeas"; }
        static const char* command_type() { return "sMN"; }
    };

    struct response : public comma::packed::packed_struct< response, 1 >
    {
        enum { type_field_size = 11 };
        static const char* type() { return "LMCstopmeas"; }
        static const char* command_type() { return "sAN"; }
        comma::packed::uint8 status_code; // 0=no error, 1=not allowed
    };
};

// todo: not sure where this belongs
// the sick document mysteriously just says "sFA x" above this table

struct sopas
{
    struct error
    {
        enum codes
        {
              ok = 0
            , methodin_access_denied=1
            , methodin_unknown_index=2
            , variable_unknown_index=3
            , local_condition_failed=4
            , invalid_data=5
            , unknown_error=6
            , buffer_overflow=7
            , buffer_underflow=8
            , error_unknown_type=9
            , variable_write_access_denied=10
            , unknown_cmd_for_nameserver=11
            , unknown_cola_command=12
            , methodin_server_busy=13
            , flex_out_of_bounds=14
            , eventreg_unknown_index=15
            , cola_a_value_overflow=16
            , cola_a_invalid_character=17
            , osai_no_message=18
            , osai_no_answer_message=19
            , internal=20
            , hub_address_corrupted=21
            , hub_address_decoding=22
            , hub_address_address_exceeded=23
            , hub_address_blank_expected=24
            , async_methods_are_suppressed=25
            , complex_arrays_not_supported=26
        };
    };
};

// todo: <STX>sSI 2 1<ETX> apparently frames every response
// todo: apparently there is an undocumented command <STX>sEN SCParmCngd 0<ETX> to deactivate these sSI 'events'

} // namespace payloads {

} } } } // namespace snark {  namespace sick { namespace cola { namespace binary {

#endif // #ifndef SNARK_SENSORS_SICK_COLA_BINARY_PACKETS_H_
