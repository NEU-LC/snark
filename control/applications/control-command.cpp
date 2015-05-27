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

#include <comma/csv/options.h>
#include <comma/csv/stream.h>
#include "control.h"
#include "pid.h"

static const char* name() { return "control-command"; }

template< typename T > std::string field_names() { return comma::join( comma::csv::names< T >( false ), ',' ); }
template< typename T > std::string format( std::string fields ) { return comma::csv::format::value< T >( fields, false ); }

static void usage( bool verbose = false )
{
    std::cerr << std::endl;
    std::cerr << "take control errors (cross_track,heading) in stdin and output velocity and turn rate to stdout" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: cat errors.csv | " << name() << " [<options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --pid=<p>,<i>,<d>,<error threshold>: pid parameters" << std::endl;
    std::cerr << std::endl;
    std::cerr << comma::csv::options::usage( field_names< snark::control::error_t >() ) << std::endl;
    std::cerr << std::endl;
    exit( 1 );
}

int main( int ac, char** av )
{
    comma::command_line_options options( ac, av, usage );
    std::vector< std::string > v = comma::split( options.value< std::string >( "--pid" ), ',' );
    if( v.size() != 3 && v.size() != 4 ) { std::cerr << "control-heading: expected pid parameters, got " << options.value< std::string >( "--pid" ) << std::endl; return 1; }
    double p = boost::lexical_cast< double >( v[0] );
    double i = boost::lexical_cast< double >( v[1] );
    double d = boost::lexical_cast< double >( v[2] );
    boost::optional< double > error_threshold;
    if( v.size() == 4 ) { error_threshold = boost::lexical_cast< double >( v[3] ); }
    snark::control::pid< snark::control::external > external_pid( p, i, d, error_threshold );
    snark::control::pid< snark::control::internal > internal_pid( p, i, d, error_threshold );
}