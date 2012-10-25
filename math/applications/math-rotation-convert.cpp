// This file is part of snark, a generic and flexible library
// for robotics research.
//
// Copyright (C) 2011 The University of Sydney
//
// snark is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// snark is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
// for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with snark. If not, see <http://www.gnu.org/licenses/>.


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

