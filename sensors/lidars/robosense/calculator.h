// This file is provided in addition to snark and is not an integral
// part of snark library.
// Copyright (c) 2018 Vsevolod Vlaskine
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
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

// snark is a generic and flexible library for robotics research
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

/// @author vsevolod vlaskine

#pragma once

#include <array>
#include <string>
#include <Eigen/Core>
#include <boost/date_time/posix_time/ptime.hpp>
#include "packet.h"

namespace snark { namespace robosense {

class calculator
{
    public:
        struct point
        {
            boost::posix_time::ptime t;
            comma::uint32 scan;
            comma::uint32 id;
            double range;
            double bearing;
            double elevation;
            comma::uint32 reflectivity;
            Eigen::Vector3d coordinates;
            
            point(): id( 0 ), scan( 0 ), range( 0 ), bearing( 0 ), elevation( 0 ), reflectivity( 0 ), coordinates( Eigen::Vector3d::Zero() ) {}
            
            bool valid() const;
        };
        
        class scan_tick
        {
            public:
                scan_tick( unsigned int max_number_of_missing_packets = 100 ); // todo: arbitrary: 10 missing packets by default will trigger a new scan and detect the current scan as incomplete
                
                std::pair< bool, bool > is_new_scan( const boost::posix_time::ptime& timestamp, const msop::packet& packet );

            private:
                boost::posix_time::time_duration max_gap_;
                boost::optional< unsigned int > last_angle_;
                boost::posix_time::ptime last_timestamp_;
                bool valid_scan_;
        };
        
        calculator();
        
        calculator( const std::string& elevation, const std::string& channel_num ); // todo: generalize to 32 beams
        
        double range( unsigned int r, unsigned int laser, unsigned int temperature ) const;
        
        ::Eigen::Vector3d to_cartesian( unsigned int laser, double range, double angle ) const;
        
        double intensity( unsigned int laser, unsigned char intensity, double distance ) const; // todo
        
        const std::array< double, robosense::msop::packet::data_t::number_of_lasers >& elevation() const { return elevation_; }
        
        point make_point( unsigned int scan, const boost::posix_time::ptime& t, const robosense::msop::packet::const_iterator& it, unsigned int temperature );
        
    private:
        std::array< double, robosense::msop::packet::data_t::number_of_lasers > elevation_;
        typedef std::array< std::array< double, 41 >, robosense::msop::packet::data_t::number_of_lasers > channel_num_t_;
        boost::optional< channel_num_t_ > channel_num_;
        struct laser_
        {
            double sin;
            double cos;
            
            laser_(): sin( 0 ), cos( 0 ) {}
            laser_( unsigned int index, const std::array< double, 16 >& elevation ) : sin( std::sin( elevation[ index ] ) ), cos( std::cos( elevation[ index ] ) ) {}
        };
        typedef std::array< laser_, robosense::msop::packet::data_t::number_of_lasers > lasers_t_;
        lasers_t_ lasers_;
        void init_lasers_();
};

} } // namespace snark { namespace robosense {
