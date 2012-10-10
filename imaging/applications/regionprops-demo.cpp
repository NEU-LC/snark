#include <snark/imaging/region_properties.h>
#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// run regionprops on an image
int main( int ac, char** av )
{
    if( ac != 2 )
    {
        std::cerr << "usage: " << av[0] << " <image> " << std::endl;
        return 1;
    }
    cv::Mat image = cv::imread( av[1], 1 );
    cv::Mat binary;
    cv::threshold( image, binary, 128, 255, CV_THRESH_BINARY_INV );
    snark::imaging::region_properties properties( binary );
    properties.show( image );
    cv::imshow( "image", image );
    cv::waitKey();
    return 0;
}
