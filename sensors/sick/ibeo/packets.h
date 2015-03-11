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


#ifndef SNARK_SENSORS_SICK_IBEO_PACKETS_H_
#define SNARK_SENSORS_SICK_IBEO_PACKETS_H_

/// @file packets.h
/// sick (ibeo) ldmrs laser communication packet layout
/// @author vsevolod vlaskine (v.vlaskine@acfr.usyd.edu.au)

#include <boost/array.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <comma/packed/byte.h>
#include <comma/packed/little_endian.h>
#include <comma/packed/big_endian.h>
#include <comma/packed/struct.h>

namespace snark {  namespace sick { namespace ibeo {

/// NTP timestamp
struct timestamp : public comma::packed::packed_struct< timestamp, 8 >
{
    comma::packed::net_uint32 seconds;
    comma::packed::net_uint32 fractions;
};

/// NTP timestamp for scan header... sigh...
struct little_endian_timestamp : public comma::packed::packed_struct< little_endian_timestamp, 8 >
{
    comma::packed::uint32 fractions;
    comma::packed::uint32 seconds;
};

/// Ibeo LD-MRS packet header
struct header : public comma::packed::packed_struct< header, 24 >
{
    /// packet types
    /// @todo test: big or little endian; make a separate type?
    enum types { scan_type = 0x2202, command_type = 0x2010, response_type = 0x2020, fault_type = 0x2030 };

    /// sentinel value
   static boost::array< unsigned char, 4 > sentinel_value;

    /// sentinel (see the value below)
    comma::packed::net_uint32 sentinel;

    /// previous message size; ignore, if in measuring mode
    comma::packed::net_uint32 previous_size;

    /// message size without header
    comma::packed::net_uint32 payload_size;

    /// padding
    comma::packed::byte padding;

    /// device id; ignore, if connected directly to the device
    comma::packed::byte device_id;

    /// message type
    comma::packed::net_uint16 type;

    /// message timestamp
    timestamp t;

    /// default constructor
    header();

    /// return true, if sentinel is valid
    bool valid() const;
};

/// fault/warning packet body
/// @todo define fault and warning bits
struct fault : public comma::packed::packed_struct< fault, 16 >
{
    comma::packed::uint16 faultRegister1;
    comma::packed::uint16 faultRegister2;
    comma::packed::uint16 warningRegister1;
    comma::packed::uint16 warningRegister2;
    boost::array< comma::packed::uint16, 4 > reserved;
    bool fatal() const;
};

std::ostream& operator<<( std::ostream&, const fault& rhs );

/// scan-related types
struct scan : public comma::packed::packed_struct< scan, 44 >
{
    /// Ibeo LD-MRS scan header (for packet of type 0x2202)
    struct header : public comma::packed::packed_struct< header, 44 >
    {
        /// sensor status values
        /// @todo test: big or little endian; make a separate type?
        enum Statuses { measurementFrequencyReached = 0x0008, externalSyncDetected = 0x0010, synced = 0x0020, syncMaster = 0x0040 };

        /// measurement number
        comma::packed::uint16 measurement_number;

        /// sensor status
        comma::packed::uint16 status;

        /// synchronisation phase offset
        comma::packed::uint16 sync_phase_offset;

        /// start time of measurement
        little_endian_timestamp start; //timestamp start;

        /// end time of measurement
        little_endian_timestamp finish; //timestamp finish;

        /// number of angular steps per scanner rotation
        comma::packed::uint16 steps;

        /// begin angle in angular steps
        /// @todo can be signed, but no signed net int! fix!
        comma::packed::uint16 start_angle;

        /// end angle in angular steps
        /// @todo can be signed, but no signed net int! fix!
        comma::packed::uint16 finish_angle;

        /// number of laser returns
        comma::packed::uint16 points_count;

        /// reserved fields
        boost::array< comma::packed::uint16, 7 > padding;
    };

    /// Ibeo LD-MRS data point
    struct point : public comma::packed::packed_struct< point, 10 >
    {
        /// layer and echo numbers
        struct id : public comma::packed::packed_struct< id, 1 >
        {
            /// value
            comma::packed::byte value;

            /// layer number
            unsigned int layer() const;

            /// echo number
            unsigned int echo() const;
        };

        /// point layer and echo numbers
        id id;

        /// point flag types
        enum { transparent = 0x01, dust = 0x02, rain = dust, noise = dust, dirt = 0x08 };

        /// point flags
        comma::packed::byte flags;

        /// azymuth in angular steps (sic, little endian)
        /// @todo can be signed, but no little endian int! fix!
        comma::packed::uint16 angle;

        /// range in cm
        comma::packed::uint16 range;

        /// echo pulse width in cm
        comma::packed::uint16 echo_pulse_width;

        /// padding
        comma::packed::uint16 padding;

        /// return elevation in radians from layer()
        double elevation() const;
    };

    /// scan header
    header scan_header;

    /// return points
    point* points();

    /// return points
    const point* points() const;

    /// take angle in steps, return angle in radians
    double angle_as_radians( short angle ) const;

    /// take angle in steps, return angle in radians
    double angle_as_radians( const point& point ) const;

    /// helper class for faster calculating point timestamps
    class timestamps
    {
        public:
            timestamps( const scan& scan );
            boost::posix_time::ptime operator[]( std::size_t index ) const;
        private:
            const scan& m_scan;
            boost::posix_time::ptime m_start;
            boost::posix_time::ptime m_finish;
            boost::posix_time::time_duration m_elapsed;
            short m_start_angle;
            short m_finish_angle;
            short m_diff;
            unsigned short m_steps;
    };
};

struct scan_packet : public comma::packed::packed_struct< scan_packet, header::size + scan::size >
{
    header packet_header;
    scan packet_scan;
};

struct commands
{
    /// command header
    struct header : public comma::packed::packed_struct< header, 4 >
    {
        comma::packed::uint16 id;
        comma::packed::net_uint16 padding;
    };

    /// response header
    struct response_header : public comma::packed::packed_struct< response_header, 2 >
    {
        comma::packed::uint16 id;
        std::size_t payload_size() const;
    };

    /// command base class
    template < typename C, std::size_t Size >
    struct command : public comma::packed::packed_struct< C, header::size + Size >
    {
        header command_header;
        command() { command_header.id = C::id; }
    };

    /// response base class
    template < typename C, std::size_t Size >
    struct response : public comma::packed::packed_struct< typename C::response, response_header::size + Size >
    {
        response_header header;
        response() { header.id = C::id; }
        void fail() { header.id = header.id() | 0x8000; }
        bool ok() const { return ( header.id() & 0x8000 ) == 0; }
        bool matches( comma::uint16 id ) { return id == ( header.id() & 0x3fff ); }
    };

    enum types
    {
          reset_dsp_type = 0x0000
        , get_status_type = 0x0001
        , save_configuration_type = 0x0004
        , set_type = 0x0010
        , get_type = 0x0011
        , reset_type = 0x001a
        , start_type = 0x0020
        , stop_type = 0x0021
        , set_ntp_seconds_type = 0x0030
        , set_ntp_fractions_type = 0x0031
    };

    /// reset DSP (no response)
    struct reset_dsp : public command< reset_dsp, 0 >
    {
        enum { id = reset_dsp_type };

        // no response
    };

    /// get sensor status
    struct get_status : public command< get_status, 0 >
    {
        enum { id = get_status_type };

        struct response : public commands::response< get_status, 30 >
        {
            comma::packed::uint16 firmwareVersion; // e.g. 0x1230 = version 1.2.3, 0x123B = version 1.2.3b
            comma::packed::uint16 fpgaVersion; // e.g. 0x1230 = version 1.2.3, 0x123B = version 1.2.3b
            comma::packed::uint16 status; // todo: define status enum
            comma::packed::uint32 reserved0;
            comma::packed::uint16 temperature; // -( Temperature - 579.2364 ) / 3.63
            comma::packed::uint16 serialNumber0; // YY CW (e. g. YY CW = 0x0740 = year '07, calendar week 40)
            comma::packed::uint16 serialNumber1; // serial number counter
            comma::packed::uint16 reserved1;
            boost::array< comma::packed::uint16, 3 > fpgaVersionDate; // cryptic: YYYY MM DD HH MM FPGA S
            boost::array< comma::packed::uint16, 3 > dspVersionDate; // cryptic: YYYY MM DD HH MM
        };
    };

    /// save configuration in the sensor's permanent memory
    struct save_configuration : public command< save_configuration, 0 >
    {
        enum { id = save_configuration_type };

        struct response : public commands::response< save_configuration, 0 > {};
    };

    /// set parameter
    struct set : public command< set, 6 >
    {
        enum { id = set_type };
        enum IndexValues { ip_address = 0x1000, tcp_port = 0x1001, subnet_mask = 0x1002, gateway = 0x1003, data_output_flag = 0x1012  }; // data output flag: 16 bit, see documentation for meaning, if we need it at all
        comma::packed::uint16 index;
        comma::packed::uint32 value; // todo: since it is little endian, it's really unclear what on the earth their documentation means

        struct response : public commands::response< set, 0 > {};
    };

    /// get parameter
    struct get : public command< get, 2 >
    {
        enum { id = get_type };
        comma::packed::uint16 index;

        struct response : public commands::response< get, 6 >
        {
            comma::packed::uint16 index;
            comma::packed::uint32 value; // todo: since it is little endian, it's really unclear what on the earth their documentation means
        };
    };

    /// reset sensor to factory settings
    struct reset : public command< reset, 0 >
    {
        enum { id = reset_type };

        struct response : public commands::response< reset, 0 > {};
    };

    /// start scanning and sending scan data ("start measurement" in terms of the ibeo documentation)
    struct start : public command< start, 0 >
    {
        enum { id = start_type };

        struct response : public commands::response< start, 0 > {};
    };

    /// stop scanning and sending scan data ("stop measurement" in terms of the ibeo documentation)
    struct stop : public command< stop, 0 >
    {
        enum { id = stop_type };

        struct response : public commands::response< stop, 0 > {};
    };

    /// set NTP time seconds
    struct set_ntp_seconds : public command< set_ntp_seconds, 6 >
    {
        enum { id = set_ntp_seconds_type };
        comma::packed::uint16 reserved; // field missed in the sick documentation
        comma::packed::uint32 seconds;

        set_ntp_seconds();
        set_ntp_seconds( comma::uint32 seconds );

        struct response : public commands::response< set_ntp_seconds, 0 > {};
    };

    /// set NTP time second fractions
    struct set_ntp_fractions : public command< set_ntp_fractions, 6 >
    {
        enum { id = set_ntp_fractions_type };
        comma::packed::uint16 reserved; // field missed in the sick documentation
        comma::packed::uint32 fractions;

        set_ntp_fractions();
        set_ntp_fractions( comma::uint32 fractions );

        struct response : public commands::response< set_ntp_fractions, 0 > {};
    };

    /// command packet
    template < typename C >
    struct packet : public comma::packed::packed_struct< packet< C >, ibeo::header::size + C::size >
    {
        ibeo::header header;
        C command;
        packet() { header.type = ibeo::header::command_type; header.payload_size = C::size; }
        packet( const C& command ) : command( command ) { header.type = ibeo::header::command_type; header.payload_size = C::size; }

        struct response : public comma::packed::packed_struct< response, ibeo::header::size + C::response::size >
        {
            ibeo::header header;
            typename C::response body;
            response() { header.type = ibeo::header::response_type; header.payload_size = C::response::size; }
            response( const typename C::response& body ) : body( body ) { header.type = ibeo::header::response_type; header.payload_size = C::response::size; }
        };
    };
};

std::ostream& operator<<( std::ostream& os, const commands::get_status::response& rhs );
std::ostream& operator<<( std::ostream& os, const commands::get::response& rhs );

} } } // namespace snark {  namespace sick { namespace ibeo {

namespace snark {  namespace sick { namespace ldmrs = ibeo; } }
    
#endif // #ifndef SNARK_SENSORS_SICK_IBEO_PACKETS_H_
