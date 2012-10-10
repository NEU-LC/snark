#ifndef SNARK_IMAGING_REGION_PROPERTIES_H
#define SNARK_IMAGING_REGION_PROPERTIES_H

#include <opencv2/core/core.hpp>

namespace snark{ namespace imaging {

/// do connected components analysis in a binary image
/// similar to the matlab function 'regionprops' 
class region_properties
{
public:
    struct blob
    {
        cv::Point centroid;
        double area;
        double majorAxis;
        double minorAxis;
        double orientation;
        double eccentricity;
        double solidity;
    };
    
    region_properties( const cv::Mat& image, double minArea = 1 );

    void show( cv::Mat& image, bool text = true );
    const std::vector< blob >& blobs() const { return m_blobs; }
    
private:
    double m_minArea;
    std::vector< blob > m_blobs;
};
    
    
} } 

#endif
