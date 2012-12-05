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


#include "parameters.h"
#include <fstream>
#include <comma/base/exception.h>
#include <comma/name_value/ptree.h>
#include <snark/math/rotation_matrix.h>

namespace snark { namespace imaging {

camera_parser::camera_parser ( const std::string& file, const std::string& path )
{
    std::ifstream ifs( file.c_str() ); // todo: config stream instead?
    if( !ifs.is_open() )
    {
        COMMA_THROW_STREAM( comma::exception, "failed to open file: " << file );
    }
    boost::property_tree::ptree tree;
    comma::property_tree::from_name_value( ifs, tree, '=', ' ' );
    comma::from_ptree from_ptree( tree, path, true );
    camera_parameters parameters;
    comma::visiting::apply( from_ptree, parameters );

    std::vector< std::string > v = comma::split( parameters.focal_length, ',' );
    assert( v.size() == 2 );
    double fX = boost::lexical_cast< double >( v[0] );
    double fY = boost::lexical_cast< double >( v[1] );

    v = comma::split( parameters.center, ',' );
    assert( v.size() == 2 );
    double cX = boost::lexical_cast< double >( v[0] );
    double cY = boost::lexical_cast< double >( v[1] );

    v = comma::split( parameters.distortion, ',' );
    assert( v.size() == 5 );
    double k1 = boost::lexical_cast< double >( v[0] );
    double k2 = boost::lexical_cast< double >( v[1] );
    double p1 = boost::lexical_cast< double >( v[2] );
    double p2 = boost::lexical_cast< double >( v[3] );
    double k3 = boost::lexical_cast< double >( v[4] );
    
    v = comma::split( parameters.rotation, ',' );
    assert( v.size() == 3 );
    double roll = boost::lexical_cast< double >( v[0] );
    double pitch = boost::lexical_cast< double >( v[1] );
    double yaw = boost::lexical_cast< double >( v[2] );

    v = comma::split( parameters.translation, ',' );
    assert( v.size() == 3 );
    double tX = boost::lexical_cast< double >( v[0] );
    double tY = boost::lexical_cast< double >( v[1] );
    double tZ = boost::lexical_cast< double >( v[2] );

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
        assert( v.size() == 2 );
        unsigned int width = boost::lexical_cast< unsigned int >( v[0] );
        unsigned int height = boost::lexical_cast< unsigned int >( v[1] );
        std::ifstream stream( parameters.map.c_str() );
        if( !stream )
        {
            COMMA_THROW( comma::exception, "failed to open undistort map in \"" << parameters.map << "\"" );
        }
        std::size_t size = width * height * 4;
        std::vector< char > buffer( size );
        
        stream.read( &buffer[0], size );
        if( stream.gcount() < 0 || std::size_t( stream.gcount() ) != size )
        {
            COMMA_THROW( comma::exception, "failed to read \"" << parameters.map << "\"" );
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

