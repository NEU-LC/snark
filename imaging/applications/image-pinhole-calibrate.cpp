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

/// @author vsevolod vlaskine

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <comma/application/command_line_options.h>
#include <comma/csv/ascii.h>
#include <comma/name_value/parser.h>
#include <comma/name_value/serialize.h>
#include "../camera/pinhole.h"
#include "../camera/traits.h"
#include "../cv_mat/serialization.h"
#include "../../visiting/eigen.h"

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

static void usage( bool verbose )
{
    std::cerr << std::endl;
    std::cerr << "perform intrinsic calibration of a camera based on a calibration pattern" << std::endl;
    std::cerr << "based on example at http://docs.opencv.org/2.4/doc/tutorials/calib3d/camera_calibration/camera_calibration.html" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: cat calibration_images.bin | image-pinhole-calibrate <options> > pinhole.json" << std::endl;
    std::cerr << "       where calibration-images.bin contains serialized calibration pattern images" << std::endl;
    if( verbose ) { std::cerr << snark::cv_mat::serialization::options::usage() << std::endl; }
    else { std::cerr << "run image-pinhole-calibrate --help --verbose for more details" << std::endl; }
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --strict: exit on the image in which calibration pattern is not found" << std::endl;
    std::cerr << "    --verbose,-v: more output" << std::endl;
    std::cerr << "    --view: view detected calibration patterns, debugging option" << std::endl;
    std::cerr << std::endl;
    std::cerr << "calibration pattern options" << std::endl;
    std::cerr << "    --max-count=<count>: max number of required calibration images; default: unlimited" << std::endl;
    std::cerr << "    --pattern-height,--height=<height>: number of inner corners per column on calibration pattern" << std::endl;
    std::cerr << "    --pattern-width,--width=<width>: number of inner corners per row on calibration pattern" << std::endl;
    std::cerr << "    --pattern-square-size,--square-size=<size>: size of a square on calibration board in user-defined metric (pixels, millimeters, etc)" << std::endl;
    std::cerr << "    --pattern=<pattern>: calibration pattern: chessboard, circles-grid, asymmetric-circles-grid" << std::endl;
    std::cerr << "                         default: chessboard" << std::endl;
    std::cerr << std::endl;
    std::cerr << "calibration options" << std::endl;
    std::cerr << "    --no-aspect-ratio: do not calibrate aspect ration, i.e. assume sensor pixels are exact squares" << std::endl;
    std::cerr << "    --no-principal-point: do not calibrate principal point, i.e. assume principal point is in the centre of the image" << std::endl;
    std::cerr << "    --no-tangential-distortion: do not calibrate tangential distortion, i.e. assume sensor plane is exactly perpendicular to camera axis" << std::endl;
    std::cerr << "    --pixel-size=<metres>: ignored if --no-aspect-ratio specified; default: 1 (dodgy?)" << std::endl;
    std::cerr << std::endl;
    std::cerr << "input options" << std::endl;
    std::cerr << "    --input=[<options>]: see --help --verbose for details" << std::endl;
    std::cerr << std::endl;
    std::cerr << "examples" << std::endl;
    std::cerr << "    run on a set of calibration images" << std::endl;
    std::cerr << "        for image in images/*.JPG ; do cv-cat --file $image ; done | image-pinhole-calibrate --pattern-height=11 --pattern-width=11 --square-size 0.1 --view --verbose" << std::endl;
    std::cerr << "    todo: more examples" << std::endl;
    std::cerr << std::endl;
    exit( 0 );
}

struct output_t
{
    snark::camera::pinhole::config_t pinhole;
    double total_average_error;
    std::vector< std::string > offsets; // quick and dirty
    output_t() : total_average_error( 0 ) {}
};

namespace comma { namespace visiting {

template <> struct traits< ::output_t >
{
    template < typename Key, class Visitor > static void visit( const Key&, const ::output_t& p, Visitor& v )
    {
        v.apply( "pinhole", p.pinhole );
        v.apply( "total_average_error", p.total_average_error );
        v.apply( "offsets", p.offsets );
    }    
};

} } // namespace comma { namespace visiting {

struct patterns { enum types { chessboard, circles_grid, asymmetric_circles_grid, invalid }; };

static std::vector< cv::Point3f > pattern_corners_( patterns::types pattern, const cv::Size& pattern_size, float square_size )
{
    std::vector< cv::Point3f > corners;
    for( int i = 0; i < pattern_size.height; ++i )
    {
        for( int j = 0; j < pattern_size.width; ++j )
        {
            corners.push_back( cv::Point3f( pattern == patterns::asymmetric_circles_grid ? ( 2 * j + i % 2 ) * square_size : j * square_size, i * square_size, 0 ) );
        }
    }
    return corners;
}

static double reprojection_error( const std::vector< std::vector< cv::Point3f > >& object_points
                                , const std::vector< std::vector< cv::Point2f > >& image_points
                                , const std::vector< cv::Mat >& rvecs
                                , const std::vector< cv::Mat >& tvecs
                                , const cv::Mat& camera_matrix
                                , const cv::Mat& distortion_coefficients
                                , std::vector< float >& per_view_errors )
{
    std::vector< cv::Point2f > image_points_2;
    int total_points = 0;
    double total_error = 0;
    per_view_errors.resize( object_points.size() );
    for( unsigned int i = 0; i < object_points.size(); ++i )
    {
        cv::projectPoints( cv::Mat( object_points[i] ), rvecs[i], tvecs[i], camera_matrix, distortion_coefficients, image_points_2 );
        double err = cv::norm( cv::Mat( image_points[i] ), cv::Mat( image_points_2 ), CV_L2 );
        unsigned int n = object_points[i].size();
        per_view_errors[i] = std::sqrt( err * err / n );
        total_error += err * err;
        total_points += n;
    }
    return std::sqrt( total_error / total_points );
}

static bool calibrate( const std::vector< cv::Point3f >& pattern_corners
                     , const cv::Size& image_size
                     , const std::vector< std::vector< cv::Point2f > >& image_points
                     , int flags
                     , cv::Mat& camera_matrix
                     , cv::Mat& distortion_coefficients
                     , std::vector< cv::Mat >& rvecs
                     , std::vector< cv::Mat >& tvecs
                     , double& total_average_error )
{
    std::vector< float > reprojection_errors;
    camera_matrix = cv::Mat::eye( 3, 3, CV_64F );
    if( flags & CV_CALIB_FIX_ASPECT_RATIO ) { camera_matrix.at< double >( 0, 0 ) = 1.0; } // why??? it already is 1.0 upon initialization
    distortion_coefficients = cv::Mat::zeros( 8, 1, CV_64F );
    std::vector< std::vector< cv::Point3f > > object_points( image_points.size(), pattern_corners );
    cv::calibrateCamera( object_points, image_points, image_size, camera_matrix, distortion_coefficients, rvecs, tvecs, flags | CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5 );
    bool ok = cv::checkRange( camera_matrix ) && cv::checkRange( distortion_coefficients );
    total_average_error = reprojection_error( object_points, image_points, rvecs, tvecs, camera_matrix, distortion_coefficients, reprojection_errors );
    return ok;
}

static Eigen::Vector3d cv_to_eigen( const cv::Mat& m ) // quick and dirty
{
    switch( m.type() )
    {
        case CV_32FC1: return Eigen::Vector3d( m.at< float >( 0 ), m.at< float >( 1 ), m.at< float >( 2 ) );
        case CV_64FC1: return Eigen::Vector3d( m.at< double >( 0 ), m.at< double >( 1 ), m.at< double >( 2 ) );
        default: std::cerr << "image-pinhole-calibrate: on outputting extrinsics: expected float (CV_32FC1) or double (CV_64FC1), got type: " << m.type() << std::endl; exit( 1 );
    }
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        bool strict = options.exists( "--strict" );
        cv::Size pattern_size( options.value< int >( "--pattern-width,--width" ), options.value< int >( "--pattern-height,--height" ) );
        std::string pattern_string = options.value< std::string >( "--pattern", "chessboard" );
        patterns::types pattern = pattern_string == "chessboard" ? patterns::chessboard
                                : pattern_string == "circles-grid" ? patterns::circles_grid
                                : pattern_string == "asymmetric-circles-grid" ? patterns::asymmetric_circles_grid
                                : patterns::invalid;
        if( pattern == patterns::invalid ) { std::cerr << "image-pinhole-calibration: expected calibration pattern, got: \"" << pattern_string << "\"" << std::endl; }
        float square_size = options.value< float >( "--pattern-square-size,--square-size", 0 ); 
        int flags = 0;
        if( options.exists( "--no-principal-point" ) ) { flags |= CV_CALIB_FIX_PRINCIPAL_POINT; }
        if( options.exists( "--no-tangential-distortion" ) ) { flags |= CV_CALIB_ZERO_TANGENT_DIST; }
        if( options.exists( "--no-aspect-ratio" ) ) { flags |= CV_CALIB_FIX_ASPECT_RATIO; }
        bool view = options.exists( "--view" );
        bool verbose = options.exists( "--verbose,-v" );
        std::vector< std::vector< cv::Point2f > > image_points;
        snark::cv_mat::serialization input( comma::name_value::parser( ';', '=' ).get< snark::cv_mat::serialization::options >( options.value< std::string >( "--input", "" ) ) );
        std::vector< cv::Point3f > pattern_corners = pattern_corners_( pattern, pattern_size, square_size );
        boost::optional< cv::Size > image_size;
        unsigned int count = 0;
        boost::optional< unsigned int > max_count = options.optional< unsigned int >( "--max-count" );
        std::vector< cv::Mat > images;
        while( !std::cin.eof() && std::cin.good() && ( !max_count || count < max_count ) )
        {
            std::pair< snark::cv_mat::serialization::header::buffer_t, cv::Mat > pair = input.read< snark::cv_mat::serialization::header::buffer_t >( std::cin );
            if( pair.second.empty() ) { break; }
            if( image_size && *image_size != pair.second.size() ) { std::cerr << "image-pinhole-calibrate: expected all calibration images of the same size; got image " << ( count - 1 ) << " of size: " << image_size->width << "," << image_size->height << " and image " << count << " of size: " << pair.second.size().width << "," << pair.second.size().height << std::endl; return 1; }
            image_size = pair.second.size();
            std::vector< cv::Point2f > points;
            bool found = pattern == patterns::chessboard ? cv::findChessboardCorners( pair.second, pattern_size, points, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE )
                       : pattern == patterns::circles_grid ? findCirclesGrid( view, pattern_size, points )
                       : pattern == patterns::asymmetric_circles_grid ? findCirclesGrid( view, pattern_size, points, cv::CALIB_CB_ASYMMETRIC_GRID )
                       : false;
            if( !found )
            {
                if( strict ) { std::cerr << "image-pinhole-calibrate: no calibration pattern found in image " << count << std::endl; return 1; }
                continue;
            }
            if( verbose ) { std::cerr << "image-pinhole-calibrate: found calibration pattern " << image_points.size() << " in image " << count << std::endl; }
            if( view ) { images.push_back( pair.second.clone() ); }
            if( pattern == patterns::chessboard ) // improve found corners
            {
                cv::Mat grey;
                cv::cvtColor( pair.second, grey, cv::COLOR_BGR2GRAY );
                cv::cornerSubPix( grey, points, cv::Size( 11, 11 ), cv::Size( -1, -1 ), cv::TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1 ) );
            }
            image_points.push_back( points );
            if( view )
            { 
                cv::drawChessboardCorners( pair.second, pattern_size, cv::Mat( points ), true );
                cv::imshow( "calibration pattern", pair.second );
                cv::waitKey( 1000 );
            }
            ++count;
        }
        if( image_points.empty() ) { std::cerr << "image-pinhole-calibrate: found no image points in " << count << " image(s)" << std::endl; return 1; }
        ::output_t output;
        cv::Mat camera_matrix;
        cv::Mat distortion_coefficients;
        std::vector< cv::Mat > rvecs;
        std::vector< cv::Mat > tvecs;
        if( !calibrate( pattern_corners, *image_size, image_points, flags, camera_matrix, distortion_coefficients, rvecs, tvecs, output.total_average_error ) ) { std::cerr << "image-pinhole-calibrate: calibration failed" << std::endl; return 1; }
        for( unsigned int i = 0; i < images.size(); ++i )
        {
            cv::Mat undistorted = images[i].clone();
            cv::undistort( images[i], undistorted, camera_matrix, distortion_coefficients );
            cv::imshow( "undistorted", undistorted );
            cv::waitKey( 2000 );
        }
        output.pinhole.image_size.x() = image_size->width;
        output.pinhole.image_size.y() = image_size->height;
        output.pinhole.principal_point = Eigen::Vector2d( camera_matrix.at< double >( 0, 2 ), camera_matrix.at< double >( 1, 2 ) );
        if( flags & CV_CALIB_FIX_ASPECT_RATIO )
        {
            output.pinhole.focal_length = camera_matrix.at< double >( 0, 0 );
        }
        else // todo: dodgy? review
        {
            double pixel_size = options.value( "--pixel-size", 1.0 );
            output.pinhole.focal_length = camera_matrix.at< double >( 0, 0 ) * pixel_size;
            output.pinhole.sensor_size = Eigen::Vector2d( pixel_size * output.pinhole.image_size.x(), output.pinhole.focal_length * output.pinhole.image_size.y() / camera_matrix.at< double >( 1, 1 ) );
        }
        output.pinhole.distortion = snark::camera::pinhole::config_t::distortion_t();
        output.pinhole.distortion->radial.k1 = distortion_coefficients.at< double >( 0 );
        output.pinhole.distortion->radial.k2 = distortion_coefficients.at< double >( 1 );
        output.pinhole.distortion->tangential.p1 = distortion_coefficients.at< double >( 2 );
        output.pinhole.distortion->tangential.p2 = distortion_coefficients.at< double >( 3 );
        output.pinhole.distortion->radial.k3 = distortion_coefficients.at< double >( 4 );
        comma::csv::ascii< std::pair< Eigen::Vector3d, Eigen::Vector3d > > ascii; // quick and dirty
        for( unsigned int i = 0; i < rvecs.size(); i++ ) { output.offsets.push_back( ascii.put( std::make_pair( cv_to_eigen( rvecs[i] ), cv_to_eigen( tvecs[i] ) ) ) ); }
        comma::write_json( output, std::cout );
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "image-pinhole-calibration: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "image-pinhole-calibration: unknown exception" << std::endl; }
    return 1;
}
