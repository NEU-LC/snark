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
// 3. All advertising materials mentioning features or use of this software
//    must display the following acknowledgement:
//    This product includes software developed by the The University of Sydney.
// 4. Neither the name of the The University of Sydney nor the
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

#include <stdint.h>
#include <boost/property_tree/json_parser.hpp>
#include <boost/numeric/conversion/cast.hpp>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/date_time/posix_time/posix_time_duration.hpp>
#include <boost/optional.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/asio.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/convenience.hpp>
#include <boost/thread.hpp>
#include <comma/application/command_line_options.h>
#include <comma/csv/stream.h>
#include <comma/base/types.h>
#include <comma/math/compare.h>
#include <comma/visiting/apply.h>
#include <comma/name_value/ptree.h>
#include <comma/name_value/parser.h>
#include <comma/io/stream.h>
#include <comma/io/publisher.h>
#include <comma/string/string.h>
#include <comma/application/signal_flag.h>
#include <comma/csv/traits.h>

namespace comma { namespace ur { 

struct config_t 
{
    config_t( const std::string& filepath )
    {
        std::ifstream config_ifs( filepath.c_str() );
        if( !config_ifs.is_open() ) { COMMA_THROW( comma::exception, "failed to open file: " + filepath ); }
        boost::property_tree::ptree t;
        boost::property_tree::read_json( config_ifs, t );
        comma::from_ptree from_ptree( t, true );
        comma::visiting::apply( from_ptree ).to( *this );
    }
    struct move_options_t { typedef double type; type acceleration; type speed; type time; type radius; };
    move_options_t move_options;
};

} } // namespace comma { namespace ur { 

namespace comma { namespace visiting {    
    
template <> struct traits< comma::ur::config_t::move_options_t >
{
    template< typename K, typename V > static void visit( const K&, comma::ur::config_t::move_options_t& t, V& v )
    {
        v.apply( "acceleration", t.acceleration );
        v.apply( "speed", t.speed );
        v.apply( "time", t.time );
        v.apply( "radius", t.radius );
    }    
    
    template< typename K, typename V > static void visit( const K&, const comma::ur::config_t::move_options_t& t, V& v )
    {
        v.apply( "acceleration", t.acceleration );
        v.apply( "speed", t.speed );
        v.apply( "time", t.time );
        v.apply( "radius", t.radius );
    }
};

template <> struct traits< comma::ur::config_t >
{
    template< typename K, typename V > static void visit( const K&, comma::ur::config_t& t, V& v )
    {
        v.apply( "move-options", t.move_options );
    }
    
    template< typename K, typename V > static void visit( const K&, const comma::ur::config_t& t, V& v )
    {
        v.apply( "move-options", t.move_options );
    }
};

} } // namespace comma { namespace visiting {
