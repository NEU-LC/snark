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

#include <cmath>
#include <fstream>
#include <boost/lexical_cast.hpp>
#include <comma/base/exception.h>
#include <comma/string/string.h>
#include "calculator.h"

namespace snark { namespace robosense {

static std::array< double, 16 > default_elevation_ = {{ -15. * M_PI / 180
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

void calculator::init_lasers_()
{
    for( unsigned int j = 0; j < robosense::msop::packet::data_t::number_of_lasers; ++j ) { lasers_[j] = laser_( j, elevation_ ); }
}
                                                      
calculator::calculator(): elevation_( default_elevation_ ) { init_lasers_(); }

calculator::calculator( const std::array< double, 16 >& elevation ): elevation_( elevation ) { init_lasers_(); }

calculator::calculator( const std::string& elevation )
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
    init_lasers_();
}

::Eigen::Vector3d calculator::point( unsigned int laser, double range, double angle ) const
{
    //return ::Eigen::Vector3d( -range * lasers_[laser].cos * std::sin( angle )
    return ::Eigen::Vector3d( range * lasers_[laser].cos * std::sin( angle )
                            , range * lasers_[laser].cos * std::cos( angle )
                            , range * lasers_[laser].sin );
}

double calculator::intensity( unsigned int, unsigned char intensity, double ) const { return intensity; }

} } // namespace snark { namespace robosense {
