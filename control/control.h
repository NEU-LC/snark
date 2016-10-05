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

#include <Eigen/Geometry>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/bimap.hpp>
#include <boost/assign.hpp>
#include <boost/optional.hpp>
#include <comma/math/cyclic.h>
#include <comma/base/exception.h>

namespace snark { namespace control {

enum mode_t { fixed, dynamic };
typedef boost::bimap< mode_t, std::string > named_mode_t;
static const named_mode_t named_modes = boost::assign::list_of< named_mode_t::relation >
    ( fixed, "fixed" )
    ( dynamic, "dynamic" );

mode_t mode_from_string( std::string s );
std::string mode_to_string( mode_t m );

static const unsigned int dimensions = 2;
typedef Eigen::Matrix< double, dimensions, 1 > vector_t;

std::string serialise( const vector_t& p );

struct wayline_t
{
public:
    wayline_t() : heading( 0 ) {} 
    wayline_t( const vector_t& from, const vector_t& to );
    bool is_past_endpoint( const vector_t& location ) const;
    double cross_track_error( const vector_t& location ) const;
    double heading_error( double yaw, double target_heading ) const;
private:
    vector_t v_;
    Eigen::Hyperplane< double, dimensions > line_;
    Eigen::Hyperplane< double, dimensions > perpendicular_line_at_end_;
public:
    double heading;
};

struct feedback_t
{
    boost::posix_time::ptime t;
    vector_t position;
    double yaw;
    double yaw_rate;
};

struct target_t
{
    target_t( bool is_absolute = false ) : heading_offset( 0 ), is_absolute( is_absolute ) {}
    target_t( const target_t& rhs ) : position( rhs.position ), heading_offset( rhs.heading_offset ), is_absolute( rhs.is_absolute ) {}
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

class wayline_follower
{
public:
    wayline_follower( mode_t mode, double proximity, bool use_past_endpoint )
        : mode_( mode )
        , proximity_( proximity )
        , use_past_endpoint_( use_past_endpoint )
        , reached_( false )
        {
            if( proximity_ < 0 ) { COMMA_THROW( comma::exception, "expected positive proximity, got " << proximity_ ); }
        }
    void set_target( const target_t& target, const vector_t& current_position );
    void update( const feedback_t& feedback );
    bool target_reached() const { return reached_; }
    error_t error() const { return error_; }
    vector_t to() const { return target_->position; }
    wayline_t wayline() const { return wayline_; }
private:
    mode_t mode_;
    double proximity_;
    bool use_past_endpoint_;
    boost::optional< target_t > target_;
    bool verbose_;
    bool reached_;
    wayline_t wayline_;
    error_t error_;
};

struct control_data_t
{
    control_data_t() {}
    control_data_t( const wayline_follower& f ) : wayline( f.wayline() ), error( f.error() ), reached( f.target_reached() ) {}
    target_t target;
    feedback_t feedback;
    wayline_t wayline;
    error_t error;
    bool reached;
};

struct command_t
{
    command_t() : turn_rate( 0 ), local_heading( 0 ) {}
    double turn_rate;
    double local_heading;
};

} } // namespace snark { namespace control

#endif // SNARK_CONTROL_H