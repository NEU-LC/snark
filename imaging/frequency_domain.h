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

#ifndef SNARK_IMAGING_FREQUENCY_DOMAIN_H
#define SNARK_IMAGING_FREQUENCY_DOMAIN_H

#include <opencv2/core/core.hpp>

namespace snark { namespace imaging {

/// filter the image in frequency domain
class frequency_domain
{
public:
    frequency_domain( const cv::Mat& mask );
    cv::Mat filter( const cv::Mat& image, const cv::Mat& mask = cv::Mat() );
    cv::Mat magnitude() const;

private:
    static void shift( cv::Mat& image );
    cv::Mat m_complexImage;
    cv::Mat m_mask;
};

} } 

#endif

