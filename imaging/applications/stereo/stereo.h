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

#ifndef SNARK_IMAGING_APPLICATIONS_STEREO_H
#define SNARK_IMAGING_APPLICATIONS_STEREO_H

#include <comma/csv/stream.h>
#include <snark/imaging/stereo/rectify_map.h>
#include <snark/imaging/stereo/point_cloud.h>
#include "parameters.h"

namespace snark { namespace imaging {

struct colored_point
{
    colored_point() : x( 0 ), y( 0 ), z( 0 ), red( 0 ), green( 0 ), blue( 0 ), block( 0 ) {}
    colored_point( double xx, double yy, double zz, unsigned char r, unsigned char g, unsigned char b ) : x( xx ), y( yy ), z( zz ), red( r ), green( g ), blue( b ), block( 0 ) {}
    boost::posix_time::ptime time;
    double x;
    double y;
    double z;
    unsigned char red;
    unsigned char green;
    unsigned char blue;
    unsigned int block;
};

/// output point cloud to stdout from stereo pair
class stereo
{
public:
    stereo( const camera_parser& left, const camera_parser& right, unsigned int width, unsigned int height, const comma::csv::options& csv );
    stereo( const camera_parser& left, const camera_parser& right,
            const cv::Mat& left_x, const cv::Mat& left_y, const cv::Mat& right_x, const cv::Mat& right_y,
            const comma::csv::options& csv );

    void process( const cv::Mat& left, const cv::Mat& right, const cv::StereoSGBM& sgbm, boost::posix_time::ptime time = boost::posix_time::ptime() );
private:
    
    Eigen::Matrix3d m_rotation;
    Eigen::Vector3d m_translation;
    rectify_map m_rectify;
    boost::scoped_ptr< comma::csv::ascii< colored_point > > m_ascii;
    boost::scoped_ptr< comma::csv::binary< colored_point > > m_binary;
    std::vector< char > m_output;
    unsigned int m_frame_counter;
};

} }

namespace comma { namespace visiting {

template <> struct traits< snark::imaging::colored_point >
{
    template < typename Key, class Visitor >
    static void visit( Key, snark::imaging::colored_point& p, Visitor& v )
    {
        v.apply( "t", p.time );
        v.apply( "x", p.x );
        v.apply( "y", p.y );
        v.apply( "z", p.z );
        v.apply( "r", p.red );
        v.apply( "g", p.green );
        v.apply( "b", p.blue );
        v.apply( "block", p.block );
    }

    template < typename Key, class Visitor >
    static void visit( Key, const snark::imaging::colored_point& p, Visitor& v )
    {
        v.apply( "t", p.time );
        v.apply( "x", p.x );
        v.apply( "y", p.y );
        v.apply( "z", p.z );
        v.apply( "r", p.red );
        v.apply( "g", p.green );
        v.apply( "b", p.blue );
        v.apply( "block", p.block );
    }
};

} }


#endif // SNARK_IMAGING_APPLICATIONS_STEREO_H
