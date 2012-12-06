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

#include "disparity.h"
#include <boost/thread/thread_time.hpp>

namespace snark { namespace imaging {

disparity::disparity ( const snark::imaging::camera_parser& left, const snark::imaging::camera_parser& right, unsigned int width, unsigned int height, const comma::csv::options& csv ):
    m_rotation( right.rotation() * left.rotation().transpose() ),
    m_translation( right.translation() - left.translation() ),
    m_rectify ( left.camera(), left.distortion(), right.camera(), right.distortion(), width, height, m_rotation, m_translation ),
    m_serialization( csv.fields, csv.format() )
{

}

disparity::disparity ( const camera_parser& left, const camera_parser& right,
                 const cv::Mat& left_x, const cv::Mat& left_y, const cv::Mat& right_x, const cv::Mat& right_y,
                 const comma::csv::options& csv ):
    m_rotation( Eigen::Matrix3d::Identity() ),
    m_translation( right.translation() - left.translation() ),
    m_rectify ( left.camera(), right.camera(), m_translation, left_x, left_y, right_x, right_y ),
    m_serialization( csv.fields, csv.format() )
{
    
}

void disparity::process( const cv::Mat& left, const cv::Mat& right, const cv::StereoSGBM& sgbm, boost::posix_time::ptime time )
{
    cv::Mat leftRectified = m_rectify.remap_left( left );
    cv::Mat rightRectified = m_rectify.remap_right( right );

    snark::imaging::point_cloud cloud( sgbm );

    cv::Mat disparity = cloud.get_disparity( leftRectified, rightRectified );
    cv::Mat disparity16;
    unsigned int numberOfDisparities = ((left.cols/8) + 15) & -16;
    disparity.convertTo( disparity16, CV_16U, 65535 / ( numberOfDisparities *16.0 ) );

    m_serialization.write( std::cout, std::make_pair( time, disparity16 ) );
}

} }


