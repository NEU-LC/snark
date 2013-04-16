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



#include <comma/application/signal_flag.h>
#include <comma/application/command_line_options.h>
#include <comma/csv/stream.h>
#include <comma/csv/options.h>
#include <snark/math/rotation_matrix.h>
#include <snark/visiting/eigen.h>

static void usage()
{
    std::cerr << std::endl;
    std::cerr << "Usage: math-rotation-convert [OPTION]" << std::endl;
    std::cerr << "convert quaternions to euler angles ( roll pitch yaw ) and back " << std::endl;
    std::cerr << "<options>" << std::endl;
    std::cerr << "    --type,-t <type>: conversion type, either rpy2q or q2rpy" << std::endl;
    std::cerr << "    --delimiter,-d <delimiter>: default ','" << std::endl;
    std::cerr << "    --field,-f <field>: field by which to split, starting from 1" << std::endl;
    std::cerr << "    --output,-o <prefix>: output prefix" << std::endl;
    std::cerr << "    --binary,-b <format>: if present, input is binary with given format" << std::endl;
    std::cerr << comma::csv::format::usage();
    std::cerr << std::endl;
    std::cerr << " examples " << std::endl;
    std::cerr << " echo \"0.182574185835,0.36514837167,0.547722557505,0.73029674334\" | math-rotation-convert --type q2rpy " << std::endl;
    std::cerr << " >> 0.785398163397,0.339836909454,1.42889927219 " << std::endl;
    std::cerr << " echo \"0.785398163397,0.339836909454,1.42889927219\" | math-rotation-convert --type rpy2q " << std::endl;
    std::cerr << " >> 0.182574185835,0.36514837167,0.547722557505,0.73029674334 " << std::endl << std::endl;
    exit( -1 );
}


comma::signal_flag shutdownFlag;

namespace snark{

/// read from std in, convert and write to std out
template< typename Input, typename Output >
class Convert
{
public:
    Convert( const comma::csv::options& options );
    bool read();

private:
    comma::csv::options m_options;
    std::string m_outputFormat;
    comma::csv::input_stream< Input > m_input;
    boost::scoped_ptr< comma::csv::ascii< Output > > m_ascii;
    boost::scoped_ptr< comma::csv::binary< Output > > m_binary;
    std::string m_buffer;
};

template< typename Input, typename Output >
Convert< Input, Output>::Convert( const comma::csv::options& options ):
    m_options( options ),
    m_outputFormat( comma::csv::format::value( Output() ) ),
    m_input( std::cin, options )
{
    if( options.binary() )
    {
        m_binary.reset( new comma::csv::binary< Output >( m_outputFormat ) );
        comma::csv::format format( m_outputFormat );
        m_buffer.resize( format.size() );
    }
    else
    {
        m_ascii.reset( new comma::csv::ascii< Output >() );
    }
}

template< typename Input, typename Output >
bool Convert< Input, Output>::read()
{
    const Input* input = m_input.read();
    if( input != NULL )
    {
        snark::rotation_matrix rotation( *input );
        Output output = rotation.convert< Output >();
        if( m_binary )
        {
            m_binary->put( output, &m_buffer[0] );
            std::cout.write( &m_buffer[0], m_buffer.size() );
        }
        else
        {
            std::vector< std::string > v;
            m_ascii->put( output, v );
            std::cout << comma::join( v, m_options.delimiter ) << std::endl;
        }
        return true;
    }
    else
    {
        return false;
    }
}

template< typename Input, typename Output >
static void run( const comma::csv::options& options )
{
    Convert< Input, Output > convert( options );

    while( !shutdownFlag && std::cin.good() && !std::cin.eof() )
    {
        convert.read();
    }
}

}

int main( int argc, char* argv[] )
{
    comma::command_line_options options( argc, argv );
    if( options.exists( "--help,-h" ) ) { usage(); }

    comma::csv::options csvOptions( options, "x,y,z,w");
    csvOptions.full_xpath = true;

    std::string type = options.value< std::string >( "--type", "" );
    if( type.empty() ) { usage(); }

    if( type == "rpy2q" )
    {
        snark::run< Eigen::Vector3d, Eigen::Quaterniond >( csvOptions );
    }
    else if( type == "q2rpy" )
    {
        snark::run< Eigen::Quaterniond, Eigen::Vector3d >( csvOptions );
    }
    else
    {
        COMMA_THROW( comma::exception, "unknown type" );
    }

    return 0;
}

