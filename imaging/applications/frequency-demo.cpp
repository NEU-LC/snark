#include <snark/imaging/frequency_domain.h>
#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// run regionprops on an image
int main( int ac, char** av )
{
    if( ac != 3 )
    {
        std::cerr << "usage: " << av[0] << " <image> <mask-size> " << std::endl;
        return 1;
    }
    cv::Mat image = cv::imread( av[1], 0 );
    cv::imshow( "image", image );
    double scale = atof( av[2] );
    unsigned int background = 255;
    unsigned int foreground = 0;
    if( scale < 0 ) // low pass instead of high pass
    {
        scale = -scale;
        background = 0;
        foreground = 255;        
    }
    cv::Mat mask( image.size(), CV_8UC1, background );
    if( image.cols * scale > 1 || image.rows * scale > 1 )
    {
        cv::ellipse( mask, cv::Point( image.cols >> 1, image.rows >> 1 ), cv::Size( image.cols * scale, image.rows * scale ), 0, 0, 360, foreground, -1 );
    }
    snark::imaging::frequency_domain frequency( mask );
    cv::Mat filtered = frequency.filter( image );
    cv::imshow( "frequency", frequency.magnitude() );
    cv::imshow( "filtered", filtered );
    cv::waitKey();
    return 0;
}

