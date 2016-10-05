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

#pragma once

#include <Eigen/Geometry>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/bimap.hpp>
#include <boost/assign.hpp>
#include <boost/optional.hpp>
#include <comma/math/cyclic.h>
#include <comma/base/exception.h>

namespace snark { namespace control {

// todo
// - fix todo comments below
// done - rename wayline_t -> wayline
// done - rename control.h/cpp -> wayline.h/cpp
// done - dimensions -> wayline::dimensions
// done - vector_t -> wayline::vector
// done - serialise: move to applications/control.h
// done - feedback_t: move to applications/control.h
// done - target_t: move to applications/control.h
// done - error_t: move to applications/control.h
// done - command_t: move to applications/control.h
// done - control_data_t: move to applications/control.h
// done - target_t, feedback_t, error_t, etc traits: move to applications/traits.h
// done - wayline_follower: move to control_error
// done - wrap_angle.h: remove, just write the expression by hand
// - feedback.reset(): remove; put feedback check in wayline_follower::update()
    
struct wayline
{
public:
    static const unsigned int dimensions = 2;
    typedef Eigen::Matrix< double, dimensions, 1 > vector;
    wayline() : heading_( 0 ) {} 
    wayline( const vector& from, const vector& to );
    bool is_past_endpoint( const vector& position ) const;
    double cross_track_error( const vector& position ) const;
    double heading_error( double current_heading, double target_heading ) const;    
    double heading() const { return heading_; }

private:
    vector v_;
    double heading_;
    Eigen::Hyperplane< double, dimensions > line_;
    Eigen::Hyperplane< double, dimensions > perpendicular_line_at_end_;
    
};

} } // namespace snark { namespace control
