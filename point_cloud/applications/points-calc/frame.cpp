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

#include <array>
#include <sstream>
#include <comma/csv/stream.h>
#include <comma/string/split.h>
#include "../../../math/rotation_matrix.h"
#include "frame.h"

namespace snark { namespace points_calc { namespace frame { namespace integrate {

std::string traits::usage()
{
    std::ostringstream oss;
    oss << "    frame-integrate (integrate-frame, deprecated)" << std::endl;
    oss << "        read reference frame on stdin, append integrated reference frame" << std::endl;
    oss << std::endl;
    oss << "        options" << std::endl;
    oss << "            --frame=[<frame>]: default frame, used as initial frame and values for skipped input fields; default: 0,0,0,0,0,0" << std::endl;
    oss << "            --from: direction of transform, default; see points-frame --help for details" << std::endl;
    oss << "            --to: direction of transform; see points-frame --help for details" << std::endl;
    oss << "            --in-place; replace input frame with the integrated one, do not append anything; convenience option" << std::endl;
    oss << std::endl;
    oss << "        examples" << std::endl;
    oss << "            the following two lines should output same result" << std::endl;
    oss << "                echo 0,0,1,0,0,0 \\" << std::endl;
    oss << "                    | points-frame --fields x,y,z,roll,pitch,yaw --from $( ( echo 0,0,0,0,1,0.5 ; echo 0,0,0,0,2,0 ) \\" << std::endl;
    oss << "                    | points-calc frame-integrate --from \\" << std::endl;
    oss << "                    | tail -n1 \\" << std::endl;
    oss << "                    | cut -d, -f7-12 )" << std::endl;
    oss << "                echo 0,0,1,0,0,0 \\" << std::endl;
    oss << "                    | points-frame --from 0,0,0,0,1,0.5 --fields x,y,z,roll,pitch,yaw \\" << std::endl;
    oss << "                    | points-frame --from 0,0,0,0,2,0 --fields x,y,z,roll,pitch,yaw" <<  std::endl;
    oss << std::endl;
    return oss.str();
}
    
std::string traits::input_fields() { return comma::join( comma::csv::names< input >( false ), ',' ); }

std::string traits::input_format() { return comma::csv::format::value< input >(); }

std::string traits::output_fields() { return comma::join( comma::csv::names< output >( true ), ',' ); }

std::string traits::output_format() { return comma::csv::format::value< output >(); }

int traits::run( const comma::command_line_options& options )
{
    comma::csv::options csv( options );
    csv.full_xpath = false;
    comma::csv::options output_csv;
    output_csv.delimiter = csv.delimiter;
    if( csv.binary() ) { output_csv.format( output_format() ); }
    snark::points_calc::pose integrated = comma::csv::ascii< input >().get( options.value< std::string >( "--frame", "0,0,0,0,0,0" ) );
    comma::csv::input_stream< input > istream( std::cin, csv, integrated );
    bool from = !options.exists( "--to" );
    std::function< void( const pose& p ) > write;
    auto run_impl = [&]() -> int
    {
        while( std::cin.good() || istream.ready() )
        {
            const input* p = istream.read();
            if( !p ) { break; }
            Eigen::Translation3d translation( integrated.coordinates );
            Eigen::Matrix3d rotation = from ? snark::rotation_matrix::rotation( integrated.orientation ) : snark::rotation_matrix::rotation( integrated.orientation ).transpose();
            Eigen::Affine3d transform = from ? ( translation * rotation ) : ( rotation.transpose() * translation.inverse() );
            integrated.coordinates = transform * p->coordinates;
            integrated.orientation = snark::rotation_matrix::roll_pitch_yaw( rotation * snark::rotation_matrix::rotation( p->orientation ) );
            write( integrated );
        }
        return 0;
    };
    if( options.exists( "--in-place" ) )
    {
        comma::csv::passed< input > passed( istream, std::cout, csv.flush );
        write = [&]( const pose& p ) { passed.write( p ); };
        return run_impl();
    }
    comma::csv::output_stream< output > ostream( std::cout, output_csv, integrated );
    comma::csv::tied< input, output > tied( istream, ostream );
    write = [&]( const pose& p ) { tied.append( p ); };
    return run_impl();
}

} } } } // namespace snark { namespace points_calc { namespace frame { namespace integrate {
