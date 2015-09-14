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

#ifndef SNARK_CONTROL_H
#define SNARK_CONTROL_H

#include <boost/date_time/posix_time/posix_time.hpp>
#include <comma/visiting/traits.h>
#include "snark/control/wayline.h"

namespace snark { namespace control {

static std::string command_app_name = "control-command";
static std::string error_app_name = "control-error";
static bool heading_offset_is_absolute_default = false;

struct feedback_t
{
    boost::posix_time::ptime t;
    vector_t position;
    double yaw;
    double yaw_rate;
};

struct target_t
{
    target_t() : heading_offset( 0 ), is_absolute( heading_offset_is_absolute_default ) {}
    vector_t position;
    double heading_offset;
    bool is_absolute;
};

struct error_t
{
    error_t() : cross_track( 0 ), heading( 0 ) {}
    double cross_track;
    double heading;
};

struct control_data_t
{
    control_data_t() {}
    control_data_t( const wayline_t& wayline, const error_t& error ) : wayline( wayline ), error( error ) {}
    target_t target;
    feedback_t feedback;
    wayline_t wayline;
    error_t error;
};

struct command_t
{
    command_t() : turn_rate( 0 ), local_heading( 0 ) {}
    double turn_rate;
    double local_heading;
};

} } // namespace snark { namespace control

namespace comma { namespace visiting {

template <> struct traits< snark::control::feedback_t >
{
    template < typename K, typename V > static void visit( const K&, snark::control::feedback_t& p, V& v )
    {
        v.apply( "t", p.t );
        v.apply( "position", p.position );
        v.apply( "yaw", p.yaw );
        v.apply( "yaw_rate", p.yaw_rate );
    }

    template < typename K, typename V > static void visit( const K&, const snark::control::feedback_t& p, V& v )
    {
        v.apply( "t", p.t );
        v.apply( "position", p.position );
        v.apply( "yaw", p.yaw );
        v.apply( "yaw_rate", p.yaw_rate );
    }
};

template <> struct traits< snark::control::target_t >
{
    template < typename K, typename V > static void visit( const K&, snark::control::target_t& p, V& v )
    {
        v.apply( "position", p.position );
        v.apply( "heading_offset", p.heading_offset );
        v.apply( "is_absolute", p.is_absolute );
    }

    template < typename K, typename V > static void visit( const K&, const snark::control::target_t& p, V& v )
    {
        v.apply( "position", p.position );
        v.apply( "heading_offset", p.heading_offset );
        v.apply( "is_absolute", p.is_absolute );
    }
};

template <> struct traits< snark::control::error_t >
{
    template < typename K, typename V > static void visit( const K&, snark::control::error_t& p, V& v )
    {
        v.apply( "cross_track", p.cross_track );
        v.apply( "heading", p.heading );
    }

    template < typename K, typename V > static void visit( const K&, const snark::control::error_t& p, V& v )
    {
        v.apply( "cross_track", p.cross_track );
        v.apply( "heading", p.heading );
    }
};

template <> struct traits< snark::control::control_data_t >
{
    template < typename K, typename V > static void visit( const K&, snark::control::control_data_t& p, V& v )
    {
        v.apply( "target", p.target );
        v.apply( "feedback", p.feedback );
        v.apply( "wayline", p.wayline );
        v.apply( "error", p.error );
    }

    template < typename K, typename V > static void visit( const K&, const snark::control::control_data_t& p, V& v )
    {
        v.apply( "target", p.target );
        v.apply( "feedback", p.feedback );
        v.apply( "wayline", p.wayline );
        v.apply( "error", p.error );
    }
};

template <> struct traits< snark::control::command_t >
{
    template < typename K, typename V > static void visit( const K&, snark::control::command_t& p, V& v )
    {
        v.apply( "turn_rate", p.turn_rate );
        v.apply( "local_heading", p.local_heading );
    }

    template < typename K, typename V > static void visit( const K&, const snark::control::command_t& p, V& v )
    {
        v.apply( "turn_rate", p.turn_rate );
        v.apply( "local_heading", p.local_heading );
    }
};

} } // namespace comma { namespace visiting {

#endif // SNARK_CONTROL_H