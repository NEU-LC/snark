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


#ifndef SNARK_ACTUATORS_QUICKSET_ptcr_COMMANDS_H_
#define SNARK_ACTUATORS_QUICKSET_ptcr_COMMANDS_H_

#include <sstream>
#include <boost/array.hpp>
#include <comma/base/exception.h>
#include <comma/packed/byte.h>
#include <comma/packed/little_endian.h>
#include <comma/packed/struct.h>

namespace snark { namespace quickset { namespace ptcr { namespace commands {

struct pan_status: public comma::packed::packed_struct< pan_status, 1 >
{
    comma::packed::byte value;

    bool cwsl() const { return 0x80 & *value.data(); }  // cw soft limit reached
    bool ccwsl() const { return 0x40 & *value.data(); } // ccw soft limit reached
    bool cwhl() const { return 0x20 & *value.data(); }  // cw hard limit reached
    bool ccwhl() const { return 0x10 & *value.data(); } // ccw hard limit reached
    bool to() const { return 0x8 & *value.data(); }     // timeout
    bool de() const { return 0x4 & *value.data(); }     // direction error
    bool fault() const { return de() || to(); }
    std::string to_string() const { std::ostringstream oss; oss << cwsl() << ccwsl() << cwhl() << ccwhl() << to() << de() << 0 << 0; return oss.str(); }
};

struct tilt_status: public comma::packed::packed_struct< tilt_status, 1 >
{
    comma::packed::byte value;

    bool usl() const { return 0x80 & *value.data(); } // up soft limit reached
    bool dsl() const { return 0x40 & *value.data(); } // down soft limit return
    bool uhl() const { return 0x20 & *value.data(); } // up hard limit reached 
    bool dhl() const { return 0x10 & *value.data(); } // down hard limit return
    bool to() const { return 0x8 & *value.data(); }   // timeout
    bool de() const { return 0x4 & *value.data(); }   // direction error
    bool fault() const { return de() || to(); }
    std::string to_string() const { std::ostringstream oss; oss << usl() << dsl() << uhl() << dhl() << to() << de() << 0 << 0; return oss.str(); }
};

struct general_status: public comma::packed::packed_struct< general_status, 1 >
{
    comma::packed::byte value;

    bool con() const { return 0x80 & *value.data(); }  // continuous rotation - limits ignored
    bool exec() const { return 0x40 & *value.data(); } // executing command
    bool des() const { return 0x20 & *value.data(); }  // returning destination coords, not current
    bool oslr() const { return 0x10 & *value.data(); } // soft limit override
    bool cwm() const { return 0x8 & *value.data(); }   // moving cw
    bool ccwm() const { return 0x4 & *value.data(); }  // moving ccw
    bool upm() const { return 0x2 & *value.data(); }   // moving up
    bool dwnm() const { return 0x1 & *value.data(); }  // moving down
    std::string to_string() const { std::ostringstream oss; oss << con() << exec() << des() << oslr() << cwm() << ccwm() << upm() << dwnm(); return oss.str(); }
};
    
struct get_status : public comma::packed::packed_struct< get_status, 7 >
{
    struct command
    {
        static const unsigned char pdir = 0x80;
        static const unsigned char tdir = 0x40;
        static const unsigned char pslo = 0x10;
        static const unsigned char tslo = 0x08;
        static const unsigned char osl = 0x04;
        static const unsigned char stop = 0x02;
        static const unsigned char res = 0x01;
    };
    
    static const char* const name;
    enum { id = 0x31 }; // static const unsigned char id = 0x31;
    comma::packed::byte command;
    comma::packed::byte pan_jog_command;
    comma::packed::byte tilt_jog_command;
    comma::packed::byte zoom1_jog;
    comma::packed::byte focus1_jog;
    comma::packed::byte zoom2_jog;
    comma::packed::byte focus2_jog;

    get_status() { ::memset( command.data(), 0, size ); }

    struct response : public comma::packed::packed_struct< get_status::response, 15 >
    {
        typedef get_status command;
        enum { id = command::id }; // static const unsigned char id = command::id;
        comma::packed::little_endian::int24 pan;
        comma::packed::little_endian::int24 tilt;
        pan_status response_pan_status;
        tilt_status response_tilt_status;
        general_status status;
        comma::packed::byte zoom1position;
        comma::packed::byte focus1position;
        comma::packed::byte zoom2position;
        comma::packed::byte focus2position;
        comma::packed::byte camera1count;
        comma::packed::byte camera2count;

        response() { camera1count = 0; camera2count = 0; } // quick and dirty: no camera for us

        bool operator==( const response& rhs ) const { return ::memcmp( data(), rhs.data(), size - 6 ) == 0; } // no camera status for now
        bool operator!=( const response& rhs ) const { return !operator==( rhs ); }
    };
};

struct move_to : public comma::packed::packed_struct< move_to, 6 >
{
    static const char* const name;
    enum { id = 0x33 }; // static const unsigned char id = 0x33;
    comma::packed::little_endian::int24 pan;
    comma::packed::little_endian::int24 tilt;

    struct response : public comma::packed::packed_struct< move_to::response, 13 >
    {
        typedef move_to command;
        enum { id = command::id }; // static const unsigned char id = command::id;
        comma::packed::little_endian::int24 pan;
        comma::packed::little_endian::int24 tilt;
        pan_status response_pan_status;
        tilt_status response_tilt_status;
        general_status status;
        comma::packed::byte zoom1position;
        comma::packed::byte focus1position;
        comma::packed::byte zoom2position;
        comma::packed::byte focus2position;
    };    
};

struct move_to_delta : public comma::packed::packed_struct< move_to_delta, 6 >
{
    static const char* const name;
    enum { id = 0x34 }; // static const unsigned char id = 0x34;
    comma::packed::little_endian::int24 pan;
    comma::packed::little_endian::int24 tilt;

    struct response : public comma::packed::packed_struct< move_to_delta::response, 13 >
    {
        typedef move_to_delta command;
        enum { id = command::id }; // static const unsigned char id = command::id;
        comma::packed::little_endian::int24 pan;
        comma::packed::little_endian::int24 tilt;
        pan_status response_pan_status;
        tilt_status response_tilt_status;
        general_status status;
        comma::packed::byte zoom1position;
        comma::packed::byte focus1position;
        comma::packed::byte zoom2position;
        comma::packed::byte focus2position;
    };
};

struct get_limits : public comma::packed::packed_struct< get_limits, 1 >
{
    static const char* const name;
    enum { id = 0x81 }; // static const unsigned char id = 0x81;
    struct direction { enum values { clockwise = 0, right = clockwise, counterclockwise = 1, left = counterclockwise, up = 2, down = 3 }; };
    static const unsigned char query = 0x80;
    comma::packed::byte direction_byte;
    get_limits() { direction_byte = query; }
    get_limits( direction::values a ) { direction_byte = query | a; }

    struct response : public comma::packed::packed_struct< response, 4 >
    {
        typedef get_limits command;
        enum { id = command::id }; // static const unsigned char id = command::id;
        comma::packed::byte direction;
        comma::packed::little_endian::int24 value;
    };
};

struct set_limits : public comma::packed::packed_struct< set_limits, 4 >
{
    static const char* const name;
    enum { id = get_limits::id }; // static const unsigned char id = get_limits::id;
    typedef get_limits::direction direction;
    comma::packed::byte direction_byte;
    comma::packed::little_endian::int24 value;
    set_limits() {}
    set_limits( direction::values a, int v ) { direction_byte = a; value = v; }

    struct response : public comma::packed::packed_struct< response, 4 >
    {
        typedef set_limits command;
        enum { id = command::id }; // static const unsigned char id = command::id;
        comma::packed::byte direction;
        comma::packed::little_endian::int24 value;
    };
};

struct set_camera : public comma::packed::packed_struct< set_camera, 2 >
{
    static const char* const name;
    enum { id = 0x64 }; // static const unsigned char id = get_limits::id;
    boost::array< comma::packed::byte, 2 > flags;
    set_camera() { flags[0] = flags[1] = 0; }

    struct response : public comma::packed::packed_struct< response, 2 >
    {
        typedef set_camera command;
        enum { id = command::id }; // static const unsigned char id = command::id;
        boost::array< comma::packed::byte, 2 > flags;
        response() { flags[0] = flags[1] = 0; }
    };
};
    
} } } } // namespace snark { namespace quickset { namespace ptcr { namespace commands {

#endif // #ifndef SNARK_ACTUATORS_QUICKSET_ptcr_COMMANDS_H_
