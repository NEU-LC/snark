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
                   const comma::csv::options& input_csv, const comma::csv::options& output_csv );

    void read();
private:
    cv::Rect m_left;
    cv::Rect m_right;
    T m_stereo;
    cv_mat::serialization m_input;
};

template< typename T >
inline stereo_stream< T >::stereo_stream ( const camera_parser& left, const camera_parser& right, const boost::array< unsigned int, 6 > roi,
                               const comma::csv::options& input_csv, const comma::csv::options& output_csv ):
    m_left( roi[0], roi[1], roi[4], roi[5] ),
    m_right( roi[2], roi[3], roi[4], roi[5] ),
    m_stereo( left, right, m_left.width, m_left.height, output_csv ),
    m_input( input_csv.fields, input_csv.format() )
{
    assert( m_left.width == m_right.width );
    assert( m_left.height == m_right.height );
}

template< typename T >
inline void stereo_stream< T >::read()
{
    std::pair< boost::posix_time::ptime, cv::Mat > image = m_input.read( std::cin );
    m_stereo.process( image.second( m_left ), image.second( m_right ), image.first );
}


} }


#endif // SNARK_IMAGING_APPLICATIONS_STEREO_STREAM_H
