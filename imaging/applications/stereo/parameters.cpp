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



#include "parameters.h"
#include <fstream>
#include <comma/base/exception.h>
//#include <comma/name_value/ptree.h>
#include <comma/name_value/serialize.h>
#include <snark/math/rotation_matrix.h>

namespace snark { namespace imaging {

camera_parser::camera_parser ( const std::string& file, const std::string& path )
{
    camera_parameters parameters;
    // TODO: leave only comma::read<> and remove the rest of try-catch block once name-value configs have been converted to json
    try { comma::read< camera_parameters >( parameters, file, path ); }
    catch( comma::exception ) { comma::read_name_value< camera_parameters >( parameters, file, path ); }

    if( parameters.focal_length == "0,0" ) { std::cerr << "stereo-to-points: warning: " << path << ": focal-length set to default (\"0,0\")" << std::endl; }
    std::vector< std::string > v = comma::split( parameters.focal_length, ',' );
    if ( v.size() != 2 ) COMMA_THROW_STREAM( comma::exception, " unexpected input for " << path << " focal-length : " << parameters.focal_length );
    double fX = boost::lexical_cast< double >( v[0] );
    double fY = boost::lexical_cast< double >( v[1] );
    // std::cerr << parameters.focal_length << " ( " << fX << "," << fY << " ) " << std::endl;

    if( parameters.center == "0,0" ) { std::cerr << "stereo-to-points: warning: " << path << ": center set to default (\"0,0\")" << std::endl; }
    v = comma::split( parameters.center, ',' );
    if ( v.size() != 2 ) COMMA_THROW_STREAM( comma::exception, " unexpected input for " << path << " center : " << parameters.center );
    double cX = boost::lexical_cast< double >( v[0] );
    double cY = boost::lexical_cast< double >( v[1] );
    // std::cerr << parameters.center << " ( " << cX << "," << cY << " ) " << std::endl;

    if( parameters.distortion == "0,0,0,0,0" ) { std::cerr << "stereo-to-points: warning: " << path << ": distortion set to default (\"0,0,0,0,0\")" << std::endl; }
    v = comma::split( parameters.distortion, ',' );
    if ( v.size() != 5 ) COMMA_THROW_STREAM( comma::exception, " unexpected input for " << path << " distortion : " << parameters.distortion );
    double k1 = boost::lexical_cast< double >( v[0] );
    double k2 = boost::lexical_cast< double >( v[1] );
    double p1 = boost::lexical_cast< double >( v[2] );
    double p2 = boost::lexical_cast< double >( v[3] );
    double k3 = boost::lexical_cast< double >( v[4] );
    // std::cerr << parameters.distortion << " ( " << k1 << "," << k2 << "," << p1 << "," << p2 << "," << k3 << " ) " << std::endl;
    
    if( parameters.rotation == "0,0,0" ) { std::cerr << "stereo-to-points: warning: " << path << ": rotation set to default (\"0,0,0\")" << std::endl; }
    v = comma::split( parameters.rotation, ',' );
    if ( v.size() != 3 ) COMMA_THROW_STREAM( comma::exception, " unexpected input for " << path << " rotation : " << parameters.rotation );
    double roll = boost::lexical_cast< double >( v[0] );
    double pitch = boost::lexical_cast< double >( v[1] );
    double yaw = boost::lexical_cast< double >( v[2] );
    // std::cerr << parameters.rotation << " ( " << roll << "," << pitch << "," << yaw << " ) " << std::endl;

    if( parameters.translation == "0,0" ) { std::cerr << "stereo-to-points: warning: " << path << ": translation set to default (\"0,0,0\")" << std::endl; }
    v = comma::split( parameters.translation, ',' );
    if ( v.size() != 3 ) COMMA_THROW_STREAM( comma::exception, " unexpected input for " << path << " translation : " << parameters.translation );
    double tX = boost::lexical_cast< double >( v[0] );
    double tY = boost::lexical_cast< double >( v[1] );
    double tZ = boost::lexical_cast< double >( v[2] );
    // std::cerr << parameters.translation << " ( " << tX << "," << tY << "," << tZ << " ) " << std::endl;

    m_camera << fX, 0,  cX,
                 0, fY, cY,
                 0, 0,  1;

    m_distortion << k1,k2,p1,p2,k3;
    snark::rotation_matrix rotation( Eigen::Vector3d( roll, pitch, yaw ) );
    m_rotation = rotation.rotation();    
    m_translation << tX, tY, tZ;

    if( !parameters.map.empty() )
    {
        v = comma::split( parameters.size, ',' );
        if ( v.size() != 2 ) COMMA_THROW_STREAM( comma::exception, " unexpected input for " << path << " map size : " << parameters.size );
        unsigned int width = boost::lexical_cast< unsigned int >( v[0] );
        unsigned int height = boost::lexical_cast< unsigned int >( v[1] );
        std::ifstream stream( parameters.map.c_str(), std::ios::binary );
        if( !stream )
        {
            COMMA_THROW( comma::exception, "failed to open undistort map in \"" << parameters.map << "\"" );
        }
        std::size_t size = width * height * 4;
        std::vector< char > buffer( size );
        
        stream.read( &buffer[0], size );
        if( stream.gcount() < 0 || std::size_t( stream.gcount() ) != size )
        {
            COMMA_THROW( comma::exception, "failed to read \"" << parameters.map << "\"" << " stream.gcount():" <<stream.gcount() );
        }
        m_map_x = cv::Mat( height, width, CV_32FC1, &buffer[0] ).clone();

        stream.read( &buffer[0], size );
        if( stream.gcount() < 0 || std::size_t( stream.gcount() ) != size )
        {
            COMMA_THROW( comma::exception, "failed to read \"" << parameters.map << "\"" );
        }
        m_map_y = cv::Mat( height, width, CV_32FC1, &buffer[0] ).clone();
        stream.peek();
        if( !stream.eof() )
        {
            COMMA_THROW( comma::exception, "expected " << ( size * 2 ) << " bytes in \"" << parameters.map << "\", got more" );
        }
    }
}



} }

