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

#include "rectified.h"
#include <boost/thread/thread_time.hpp>
#include <opencv/cv.h>

namespace snark { namespace imaging {

rectified::rectified ( const snark::imaging::camera_parser& left, const snark::imaging::camera_parser& right, unsigned int width, unsigned int height, const comma::csv::options& csv, bool input_rectified ):
    m_rotation( right.rotation() * left.rotation().transpose() ),
    m_translation( right.translation() - left.translation() ),
    m_rectify( left.camera(), left.distortion(), right.camera(), right.distortion(), width, height, m_rotation, m_translation ),
    m_input_rectified( input_rectified ), 
    m_serialization( csv.fields, csv.format() )
{

}

rectified::rectified ( const camera_parser& left, const camera_parser& right,
                 const cv::Mat& left_x, const cv::Mat& left_y, const cv::Mat& right_x, const cv::Mat& right_y,
                 const comma::csv::options& csv, bool input_rectified ):
    m_rotation( Eigen::Matrix3d::Identity() ),
    m_translation( right.translation() - left.translation() ),
    m_rectify ( left.camera(), right.camera(), m_translation, left_x, left_y, right_x, right_y ),
    m_input_rectified( input_rectified ), 
    m_serialization( csv.fields, csv.format() )
{
    
}

cv::Mat rectified::concatenate( const cv::Mat& left, const cv::Mat& right )
{
    cv::Size sz_left = left.size();
    cv::Size sz_right = right.size();
    cv::Mat concatenated( sz_left.height, sz_left.width + sz_right.width, left.type() );
    cv::Mat left_roi = cv::Mat( concatenated, cv::Rect(0, 0,  left.cols, left.rows) );
    cv::Mat right_roi = cv::Mat( concatenated, cv::Rect(left.cols, 0,  right.cols, right.rows) );
    left.copyTo( left_roi );
    right.copyTo( right_roi );
    return concatenated;
}

void rectified::process( const cv::Mat& left, const cv::Mat& right, const cv::StereoSGBM& sgbm, boost::posix_time::ptime time )
{
    if (!m_input_rectified)
    {
        cv::Mat leftRectified = m_rectify.remap_left( left );
        cv::Mat rightRectified = m_rectify.remap_right( right );
        m_serialization.write( std::cout, std::make_pair( time, concatenate( leftRectified, rightRectified ) ) );
    }
    else
    {
        m_serialization.write( std::cout, std::make_pair( time, concatenate( left, right ) ) );
    }
}

} }
