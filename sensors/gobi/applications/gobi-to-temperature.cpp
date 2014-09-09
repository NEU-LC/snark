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


#include <boost/program_options.hpp>
#include <boost/scoped_ptr.hpp>
#include <comma/application/signal_flag.h>
#include <comma/base/exception.h>
#include <comma/csv/stream.h>
#include <comma/name_value/map.h>
#include <snark/imaging/cv_mat/pipeline.h>
#include <snark/sensors/gobi/gobi.h>
#include "XFilters.h"

int main( int argc, char** argv )
{
    try
    {
        boost::program_options::options_description description( "options" );
        description.add_options()
            ( "help,h", "display help message" )
            ( "verbose,v", "be more verbose" )
            ( "celsius", "convert to degrees Celsius" )
            ( "kelvin", "convert to degrees Kelvin" )
            ( "calibration", boost::program_options::value< std::string >(), "location of the calibration file" );
        boost::program_options::variables_map vm;
        boost::program_options::store( boost::program_options::parse_command_line( argc, argv, description), vm );
        boost::program_options::parsed_options parsed = boost::program_options::command_line_parser(argc, argv).options( description ).allow_unregistered().run();
        boost::program_options::notify( vm );
        if ( vm.count( "help" ) )
        {
            std::cerr << "takes from stdin binary images produced by xenics gobi camera (16 bit grey scale)" << std::endl;
            std::cerr << "and uses the camera's calibration to output to stdout pixel values in degrees (Celsius or Kelvin)" << std::endl;
            std::cerr << std::endl;
            std::cerr << description << std::endl;
            return 1;
        }
        bool verbose = vm.count( "verbose" );
        if ( !vm.count( "calibration" ) ) { COMMA_THROW( comma::exception, "please provide the calibration file, --calibration is not specified"); }
        if ( vm.count( "celsius" ) && vm.count( "kelvin" ) ) { COMMA_THROW( comma::exception, "please specify either --celsius or --kelvin" ); }
        if ( !vm.count( "celsius" ) && !vm.count( "kelvin" ) ) { COMMA_THROW( comma::exception, "please specify --celsius or --kelvin" ); }
        std::string calibration_file = vm["calibration"].as< std::string >();
        XCHANDLE handle = 0;
        if( XC_LoadCalibration(handle, calibration_file.c_str(), XLC_StartSoftwareCorrection ) != I_OK )
        {
            COMMA_THROW( comma::exception, "failed to load calibration file \"" << calibration_file << "\"" ); 
        }
        else
        {
            if ( verbose ) { std::cerr << "gobi-to-temperature: calibration loaded" << std::endl; }
        }
        return 0;
    }
    catch( std::exception& ex )
    {
        std::cerr << argv[0] << ": " << ex.what() << std::endl;
    }
    catch( ... )
    {
        std::cerr << argv[0] << ": unknown exception" << std::endl;
    }
    return 1;
}