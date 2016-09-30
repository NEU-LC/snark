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
#include <comma/math/cyclic.h>

namespace snark { namespace control {

static const unsigned int dimensions = 2;
typedef Eigen::Matrix< double, dimensions, 1 > vector_t;

std::string serialise( const vector_t& p );

struct wayline_t
{
public:
    wayline_t() : heading( 0 ) {} 
    wayline_t( const vector_t& start, const vector_t& end );
    bool is_past_endpoint( const vector_t& location ) const;
    double cross_track_error( const vector_t& location ) const;
    double heading_error( double yaw, double heading_offset ) const;
private:
    vector_t v;
    Eigen::Hyperplane< double, dimensions > line;
    Eigen::Hyperplane< double, dimensions > perpendicular_line_at_end;
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
    control_data_t( const wayline_t& wayline, const error_t& error, bool reached ) : wayline( wayline ), error( error ), reached( reached ) {}
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