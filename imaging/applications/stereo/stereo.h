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


#ifndef SNARK_IMAGING_APPLICATIONS_STEREO_H
#define SNARK_IMAGING_APPLICATIONS_STEREO_H

#include <comma/csv/stream.h>
#include "../../stereo/rectify_map.h"
#include "../../stereo/point_cloud.h"
#include "parameters.h"

namespace snark { namespace imaging {

struct colored_point
{
    colored_point() : i( 0 ), j( 0 ), disparity( 0 ), x( 0 ), y( 0 ), z( 0 ), red( 0 ), green( 0 ), blue( 0 ), block( 0 ) {}
    colored_point( unsigned int ii, unsigned int jj, short int d, double xx, double yy, double zz, unsigned char r, unsigned char g, unsigned char b ) : i( ii ), j( jj ), disparity( d ), x( xx ), y( yy ), z( zz ), red( r ), green( g ), blue( b ), block( 0 ) {}
    boost::posix_time::ptime time;
    unsigned int i;
    unsigned int j;
    short int disparity;
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
    stereo( const camera_parser& left, const camera_parser& right, unsigned int width, unsigned int height, const comma::csv::options& csv, bool input_rectified );
    stereo( const camera_parser& left, const camera_parser& right,
            const cv::Mat& left_x, const cv::Mat& left_y, const cv::Mat& right_x, const cv::Mat& right_y,
            const comma::csv::options& csv, bool input_rectified );

    void process( const cv::Mat& left, const cv::Mat& right, const cv::StereoSGBM& sgbm, boost::posix_time::ptime time = boost::posix_time::ptime() );
private:
    
    Eigen::Matrix3d m_rotation;
    Eigen::Vector3d m_translation;
    rectify_map m_rectify;
    bool m_input_rectified;
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
        v.apply( "i", p.i );
        v.apply( "j", p.j );
        v.apply( "disparity", p.disparity );
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
        v.apply( "i", p.i );
        v.apply( "j", p.j );
        v.apply( "disparity", p.disparity );
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
