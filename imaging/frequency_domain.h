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

