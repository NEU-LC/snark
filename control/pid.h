// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2015 The University of Sydney
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

#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/optional.hpp>

namespace snark { namespace control {

class pid
{
public:
    pid();
    virtual ~pid() {}
    pid( double p, double i, double d );
    pid( double p, double i, double d, double threshold );
    double operator()( double error, const boost::posix_time::ptime& t = boost::posix_time::not_a_date_time );
    double operator()( double error, double derivative, const boost::posix_time::ptime& t = boost::posix_time::not_a_date_time );
    void reset();

protected:
    double p;
    double i;
    double d;
    double integral;
    boost::optional< double > threshold;
    boost::posix_time::ptime time;
    boost::optional< double > previous_error;
    boost::optional< double > get_time_increment( const boost::posix_time::ptime& t );
    void clip_integral();
    virtual double update_( double error, double derivative, const boost::posix_time::ptime& t, boost::optional< double > dt );
    virtual double derivative_( double error, double dt );
};

class angular_pid : public pid
{
    public:
        angular_pid( double p, double i, double d );
        angular_pid( double p, double i, double d, double threshold );

    private:
        double update_( double error, double derivative, const boost::posix_time::ptime& t, boost::optional< double > dt );
        double derivative_( double error, double dt );
};

} } // namespace snark { namespace control {
