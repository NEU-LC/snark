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


#ifndef SNARK_IMAGING_APPLICATIONS_STEREO_STREAM_H
#define SNARK_IMAGING_APPLICATIONS_STEREO_STREAM_H

#include "stereo.h"
#include <snark/imaging/cv_mat/serialization.h>

namespace snark { namespace imaging {

/// read stereo images from stdin, output point cloud or disparity to stdout
template< typename T >
class stereo_stream
{
public:
    stereo_stream( const camera_parser& left, const camera_parser& right, const boost::array< unsigned int, 6 > roi,
                   const comma::csv::options& input_csv, const comma::csv::options& output_csv, bool input_rectified = false );

    stereo_stream( const camera_parser& left, const camera_parser& right, const boost::array< unsigned int, 6 > roi,
            const cv::Mat& left_x, const cv::Mat& left_y, const cv::Mat& right_x, const cv::Mat& right_y,
            const comma::csv::options& input_csv, const comma::csv::options& output_csv, bool input_rectified = false );

    void read( const cv::StereoSGBM& sgbm );
private:
    cv::Rect m_left;
    cv::Rect m_right;
    T m_stereo;
    cv_mat::serialization m_input;
};

template< typename T >
inline stereo_stream< T >::stereo_stream ( const camera_parser& left, const camera_parser& right, const boost::array< unsigned int, 6 > roi,
                               const comma::csv::options& input_csv, const comma::csv::options& output_csv, bool input_rectified ):
    m_left( roi[0], roi[1], roi[4], roi[5] ),
    m_right( roi[2], roi[3], roi[4], roi[5] ),
    m_stereo( left, right, m_left.width, m_left.height, output_csv, input_rectified ),
    m_input( input_csv.fields, input_csv.format() )
{
    assert( m_left.width == m_right.width );
    assert( m_left.height == m_right.height );
}

template< typename T >
inline stereo_stream< T >::stereo_stream ( const camera_parser& left, const camera_parser& right, const boost::array< unsigned int, 6 > roi,
                                           const cv::Mat& left_x, const cv::Mat& left_y, const cv::Mat& right_x, const cv::Mat& right_y,
                                           const comma::csv::options& input_csv, const comma::csv::options& output_csv, bool input_rectified ):
    m_left( roi[0], roi[1], roi[4], roi[5] ),
    m_right( roi[2], roi[3], roi[4], roi[5] ),
    m_stereo( left, right, left_x, left_y, right_x, right_y, output_csv, input_rectified ),
    m_input( input_csv.fields, input_csv.format() )
{
    assert( m_left.width == m_right.width );
    assert( m_left.height == m_right.height );
}

template< typename T >
inline void stereo_stream< T >::read( const cv::StereoSGBM& sgbm )
{
    std::pair< boost::posix_time::ptime, cv::Mat > image = m_input.read( std::cin );
    m_stereo.process( image.second( m_left ), image.second( m_right ), sgbm, image.first );
}


} }


#endif // SNARK_IMAGING_APPLICATIONS_STEREO_STREAM_H
