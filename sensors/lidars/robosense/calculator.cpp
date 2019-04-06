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

#include <iostream>
#include <array>
#include <cmath>
#include <fstream>
#include <boost/lexical_cast.hpp>
#include <comma/base/exception.h>
#include <comma/csv/stream.h>
#include <comma/string/string.h>
#include <comma/visiting/traits.h>
#include "calculator.h"

namespace snark { namespace robosense {

struct channel_num { std::array< double, 41 > data; };

} } // namespace snark { namespace robosense {

namespace comma { namespace visiting {

template <> struct traits< snark::robosense::channel_num >
{
    template < typename K, typename V > static void visit( const K& k,       snark::robosense::channel_num& t, V& v ) { v.apply( "data", t.data ); }
    template < typename K, typename V > static void visit( const K& k, const snark::robosense::channel_num& t, V& v ) { v.apply( "data", t.data ); }
};
    
} } // namespace comma { namespace visiting {

namespace snark { namespace robosense {
    
calculator::scan_tick::scan_tick( unsigned int max_number_of_missing_packets ): valid_scan_( true ), max_gap_( boost::posix_time::microseconds( msop::packet::data_t::block::firing_interval() * 1000000 ) * max_number_of_missing_packets ) {}

std::pair< bool, bool > calculator::scan_tick::is_new_scan( const boost::posix_time::ptime& timestamp, const msop::packet& packet )
{
    bool tick = false;
    unsigned int angle = packet.data.blocks[0].azimuth(); // ( ... + ( 36000 - 9000 ) + 9000 ) % 36000;
    if( last_angle_ && angle < *last_angle_ ) { tick = true; valid_scan_ = true; }
    last_angle_ = angle;
    if( !timestamp.is_not_a_date_time() && !last_timestamp_.is_not_a_date_time() && timestamp - last_timestamp_ > max_gap_ ) { tick = true; valid_scan_ = false; }
    if( !timestamp.is_not_a_date_time() ) { last_timestamp_ = timestamp; }
    return std::make_pair( tick, valid_scan_ );
}

static std::array< double, robosense::msop::packet::data_t::number_of_lasers > default_elevation_ = {{ -15. * M_PI / 180
                                                                                                    , -13. * M_PI / 180
                                                                                                    , -11. * M_PI / 180
                                                                                                    ,  -9. * M_PI / 180
                                                                                                    ,  -7. * M_PI / 180
                                                                                                    ,  -5. * M_PI / 180
                                                                                                    ,  -3. * M_PI / 180
                                                                                                    ,  -1. * M_PI / 180
                                                                                                    ,  15. * M_PI / 180
                                                                                                    ,  13. * M_PI / 180
                                                                                                    ,  11. * M_PI / 180
                                                                                                    ,   9. * M_PI / 180
                                                                                                    ,   7. * M_PI / 180
                                                                                                    ,   5. * M_PI / 180
                                                                                                    ,   3. * M_PI / 180
                                                                                                    ,   1. * M_PI / 180 }};

// static std::array< std::array< double, 41 >, robosense::msop::packet::data_t::number_of_lasers > default_channel_num_ =
// {{
//     {{ 454,454,454,454,454,454,454,454,455,454,456,455,457,457,456,456,456,456,457,457,458,459,459,460,461,462,462,463,463,463,463,463,463,463,463,463,463,463,463,463,463 }},
//     {{ 459,459,459,459,459,459,459,459,459,459,460,459,461,460,460,460,460,460,461,462,463,463,464,465,465,466,467,467,467,467,467,467,467,467,467,467,467,467,467,467,467 }},
//     {{ 450,450,450,450,450,450,450,450,451,451,451,451,452,452,453,453,454,454,455,456,456,457,458,459,459,460,461,462,462,462,462,462,462,462,462,462,462,462,462,462,462 }},
//     {{ 451,451,451,451,451,451,451,451,452,451,452,452,452,453,453,453,454,454,455,456,457,458,459,460,460,461,462,463,463,463,463,463,463,463,463,463,463,463,463,463,463 }},
//     {{ 452,452,452,452,452,452,452,452,452,452,453,452,454,454,455,455,456,453,452,452,453,454,454,455,456,457,458,457,457,457,457,457,457,457,457,457,457,457,457,457,457 }},
//     {{ 451,451,451,451,451,451,451,451,451,451,452,451,453,452,453,453,454,452,451,451,452,452,453,454,455,456,456,456,456,456,456,456,456,456,456,456,456,456,456,456,456 }},
//     {{ 462,462,462,462,462,462,462,462,463,463,464,463,465,465,465,466,466,464,463,463,464,465,465,466,467,468,469,468,468,468,468,468,468,468,468,468,468,468,468,468,468 }},
//     {{ 461,461,461,461,461,461,461,461,462,461,462,462,464,463,464,464,465,463,462,462,463,464,465,466,467,467,468,467,467,467,467,467,467,467,467,467,467,467,467,467,467 }},
//     {{ 452,452,452,452,452,452,452,452,452,452,453,453,453,454,454,453,450,449,449,450,451,452,452,453,453,451,451,451,451,451,451,451,451,451,451,451,451,451,451,451,451 }},
//     {{ 467,467,467,467,467,467,467,467,468,467,468,468,469,469,470,468,466,465,465,466,467,468,468,468,469,467,467,468,468,468,468,468,468,468,468,468,468,468,468,468,468 }},
//     {{ 454,454,454,454,454,454,454,454,455,455,455,456,456,457,458,458,459,459,459,459,459,460,460,461,462,463,464,464,464,464,464,464,464,464,464,464,464,464,464,464,464 }},
//     {{ 461,461,461,461,461,461,461,461,461,461,462,462,463,463,464,464,465,466,466,466,466,467,467,468,469,469,470,471,471,471,471,471,471,471,471,471,471,471,471,471,471 }},
//     {{ 450,450,450,450,450,450,450,450,451,450,451,450,452,451,452,453,454,454,455,456,457,458,458,459,460,459,458,458,458,458,458,458,458,458,458,458,458,458,458,458,458 }},
//     {{ 455,455,455,455,455,455,455,455,455,455,456,455,457,457,457,458,459,459,460,462,463,464,463,464,465,465,464,463,463,463,463,463,463,463,463,463,463,463,463,463,463 }},
//     {{ 459,459,459,459,459,459,459,459,460,460,461,460,462,461,462,463,464,464,465,467,468,469,468,470,471,471,469,469,469,469,469,469,469,469,469,469,469,469,469,469,469 }},
//     {{ 450,450,450,450,450,450,450,450,451,451,452,452,453,453,454,454,455,455,456,458,459,460,460,461,462,461,460,459,459,459,459,459,459,459,459,459,459,459,459,459,459 }}
// }};
                                                      
void calculator::init_lasers_()
{
    for( unsigned int j = 0; j < robosense::msop::packet::data_t::number_of_lasers; ++j ) { lasers_[j] = laser_( j, elevation_ ); }
}
                                                      
calculator::calculator(): elevation_( default_elevation_ ) { init_lasers_(); }

calculator::calculator( const std::array< double, robosense::msop::packet::data_t::number_of_lasers >& elevation ): elevation_( elevation ) { init_lasers_(); }

calculator::calculator( const std::string& elevation, const std::string& channel_num = "" )
{
    if( elevation.empty() )
    {
        elevation_ = default_elevation_;
    }
    else
    {
        std::ifstream ifs( elevation );
        if( !ifs.is_open() ) { COMMA_THROW( comma::exception, "failed to open '" << elevation << "'" ); }
        unsigned int i = 0;
        for( ; !ifs.eof() && ifs.good() && i < 16; )
        {
            std::string line;
            std::getline( ifs, line );
            line = comma::strip( line );
            if( line.empty() ) { continue; }
            elevation_[i] = boost::lexical_cast< double >( line ) * M_PI / 180;
            ++i;
        }
        if( i < 16 ) { COMMA_THROW( comma::exception, "expected 16 elevation angle values in '" << elevation << "'; got only " << i ); }
    }
    init_lasers_();
    if( !channel_num.empty() )
    {
        std::ifstream ifs( channel_num );
        if( !ifs.is_open() ) { COMMA_THROW( comma::exception, "failed to open '" << channel_num << "'" ); }
        comma::csv::input_stream< robosense::channel_num > is( ifs );
        channel_num_ = channel_num_t_();
        for( unsigned int i = 0; !ifs.eof() && ifs.good() && i < 16; ++i )
        {
            const auto* p = is.read();
            if( !p ) { COMMA_THROW( comma::exception, "expected 16 channel num arrays in '" << channel_num << "'; got only " << i ); }
            ( *channel_num_ )[i] = p->data;
        }
    }
}

double calculator::range( unsigned int r, unsigned int laser, unsigned int temperature ) const { return 0.01 * ( channel_num_ ? r - ( *channel_num_ )[laser][temperature] : r ); }

::Eigen::Vector3d calculator::to_cartesian( unsigned int laser, double range, double angle ) const
{
    return ::Eigen::Vector3d( range * lasers_[laser].cos * std::sin( angle )
                            , range * lasers_[laser].cos * std::cos( angle )
                            , range * lasers_[laser].sin );
}

double calculator::intensity( unsigned int, unsigned char intensity, double ) const { return intensity; }

calculator::point calculator::make_point( unsigned int scan, const boost::posix_time::ptime& t, const robosense::msop::packet::const_iterator& it, unsigned int temperature )
{
    calculator::point p;
    p.scan = scan;
    p.t = t + boost::posix_time::microseconds( it->delay * 1000000 );
    p.id = it->id;
    p.range = range( it->range, it->id, temperature );
    p.bearing = it->azimuth;
    p.elevation = elevation()[ it->id ];
    p.reflectivity = it->reflectivity;
    p.coordinates = to_cartesian( it->id, p.range, p.bearing );
    return p;
}

} } // namespace snark { namespace robosense {
