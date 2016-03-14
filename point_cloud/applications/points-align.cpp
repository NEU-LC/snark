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

/// @author dave jennings

#include <cmath>
#include <comma/application/verbose.h>
#include <comma/csv/options.h>
#include <comma/csv/stream.h>
#include "../../math/applications/frame.h"

typedef std::pair< Eigen::Vector3d, Eigen::Vector3d > point_pair_t;

static const std::string default_fields = comma::join( comma::csv::names< point_pair_t >(), ',' );
static const std::string output_fields = comma::join( comma::csv::names< snark::applications::position >(), ',' );

static void usage( bool verbose = false )
{
    std::cerr << std::endl;
    std::cerr << "give the transform that will align a point cloud with a reference point cloud" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: " << comma::verbose.app_name() << " [<options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "default input fields: " << default_fields << std::endl;
    std::cerr << std::endl;
    std::cerr << "The input data is assumed to consist of matched points from the reference" << std::endl;
    std::cerr << "point cloud and the point cloud to be aligned. The output is a transform" << std::endl;
    std::cerr << "in the form " << output_fields << " (angles in radians) that can be given to" << std::endl;
    std::cerr << "points-frame to align the data." << std::endl;
    std::cerr << std::endl;
    if( verbose )
    {
        std::cerr << "csv stream options: " << std::endl;
        std::cerr << comma::csv::options::usage() << std::endl;
        std::cerr << std::endl;
    }
    std::cerr << "options: " << std::endl;
    std::cerr << "    --help,-h; show this help, --help --verbose for more help" << std::endl;
    std::cerr << "    --verbose,-v; more output" << std::endl;
    std::cerr << "    --output-fields: show output fields and exit" << std::endl;
    std::cerr << "    --output-format: show output format and exit" << std::endl;
    std::cerr << "    --include-error: include the error estimate in output" << std::endl;
    std::cerr << std::endl;
    std::cerr << "examples: " << std::endl;
    std::cerr << "    -- output the " << output_fields << " transform --" << std::endl;
    std::cerr << "    cat combined-points.csv | points-align" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    -- align the point cloud to the reference point cloud --" << std::endl;
    std::cerr << "    frame=$( cat combined-points.csv | points-align )" << std::endl;
    std::cerr << "    cat points.csv | points-frame --from $frame --fields=x,y,z" << std::endl;
    std::cerr << std::endl;
    exit( 0 );
}

struct position_with_error
{
    snark::applications::position position;
    double error;

    position_with_error() : error( 0 ) {}
    position_with_error( const snark::applications::position& position, double error )
        : position( position )
        , error( error )
    {}
};

namespace comma { namespace visiting {

template <> struct traits< position_with_error >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, position_with_error& p, Visitor& v )
    {
        v.apply( "x", p.position.coordinates.x() );
        v.apply( "y", p.position.coordinates.y() );
        v.apply( "z", p.position.coordinates.z() );
        v.apply( "roll", p.position.orientation.x() );
        v.apply( "pitch", p.position.orientation.y() );
        v.apply( "yaw", p.position.orientation.z() );
        v.apply( "error", p.error );
    }

    template < typename Key, class Visitor >
    static void visit( const Key&, const position_with_error& p, Visitor& v )
    {
        v.apply( "x", p.position.coordinates.x() );
        v.apply( "y", p.position.coordinates.y() );
        v.apply( "z", p.position.coordinates.z() );
        v.apply( "roll", p.position.orientation.x() );
        v.apply( "pitch", p.position.orientation.y() );
        v.apply( "yaw", p.position.orientation.z() );
        v.apply( "error", p.error );
    }
};

} } // namespace comma { namespace visiting {

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        comma::csv::options csv( options );
        csv.full_xpath = true;
        if( csv.fields.empty() ) { csv.fields = default_fields; }

        if( options.exists( "--output-fields" ))
        {
            std::cout << comma::join( options.exists( "--include-error" )
                                    ? comma::csv::names< position_with_error >()
                                    : comma::csv::names< snark::applications::position >(),
                                    ',' ) << std::endl;
            return 0;
        }

        if( options.exists( "--output-format" ))
        {
            std::cout <<
                ( options.exists( "--include-error" )
                ? comma::csv::format::value< position_with_error >()
                : comma::csv::format::value< snark::applications::position >() )
                      << std::endl;
            return 0;
        }

        Eigen::MatrixXd src( 3, 0 );
        Eigen::MatrixXd tgt( 3, 0 );

        comma::csv::input_stream< point_pair_t > istream( std::cin, csv );
        while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
        {
            const point_pair_t* p = istream.read();
            if( !p ) break;

            tgt.conservativeResize( Eigen::NoChange, tgt.cols()+1 );
            src.conservativeResize( Eigen::NoChange, src.cols()+1 );
            
            tgt(0, tgt.cols()-1) = p->first(0);
            tgt(1, tgt.cols()-1) = p->first(1);
            tgt(2, tgt.cols()-1) = p->first(2);

            src(0, src.cols()-1) = p->second(0);
            src(1, src.cols()-1) = p->second(1);
            src(2, src.cols()-1) = p->second(2);
        }

        comma::verbose << "Loaded " << tgt.cols() << " pairs of points" << std::endl;

        Eigen::Matrix4d estimate = Eigen::umeyama( src, tgt );

        comma::verbose << "umeyama estimate\n" << estimate << std::endl;

        Eigen::Matrix3d rotation( estimate.block< 3, 3 >( 0, 0 ));
        Eigen::Vector3d translation( estimate.block< 3, 1 >( 0, 3 ));

        Eigen::Vector3d orientation( snark::rotation_matrix::roll_pitch_yaw( rotation ));

        comma::verbose << "rotation matrix\n" << rotation << std::endl;
        comma::verbose << "translation ( " << comma::join( translation, ',' ) << " )" << std::endl;
        comma::verbose << "orientation (rad) ( " << comma::join( orientation, ',' ) << " )" << std::endl;
        comma::verbose << "orientation (deg) ( " << comma::join( orientation * 180 / M_PI, ',' ) << " )" << std::endl;

        snark::applications::position position( translation, orientation );
        comma::csv::options output_csv;

        if( ! options.exists( "--include-error" ))
        {
            comma::csv::output_stream< snark::applications::position > ostream( std::cout, output_csv );
            ostream.write( position );
        }
        else
        {
            // Convert src and tgt to a form that allows multiplication by estimate.
            // Change each vector from size 3 to size 4 and store 1 in the fourth row.
            src.conservativeResize( src.rows()+1, Eigen::NoChange );
            src.row( src.rows()-1 ) = Eigen::MatrixXd::Constant( 1, src.cols(), 1 );

            tgt.conservativeResize( tgt.rows()+1, Eigen::NoChange );
            tgt.row( tgt.rows()-1 ) = Eigen::MatrixXd::Constant( 1, tgt.cols(), 1 );

            const double error = ( estimate * src - tgt ).norm() / tgt.norm();

            comma::csv::output_stream< position_with_error > ostream( std::cout, output_csv );
            const position_with_error pe( position, error );
            ostream.write( pe );
        }
    }
    catch( std::exception& ex )
    {
        std::cerr << comma::verbose.app_name() << ": " << ex.what() << std::endl;
    }
    catch( ... )
    {
        std::cerr << comma::verbose.app_name() << ": unknown exception" << std::endl;
    }
    return 0;
}
