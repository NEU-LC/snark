#include <cmath>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/adapted/boost_tuple.hpp>
#include <boost/foreach.hpp>
#include <Eigen/Core>
#include <snark/imaging/region_properties.h>
#include <comma/base/exception.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

BOOST_GEOMETRY_REGISTER_BOOST_TUPLE_CS( cs::cartesian )

namespace snark{ namespace imaging {

/// compute the area of a polygon and its convex hull
/// @param points polygon points
/// @param area area of the polygon
/// @param convexArea area of the convex hull
void compute_area( const std::vector< cv::Point >& points, double& area, double& convexArea )
{
    boost::geometry::model::polygon< boost::tuple< double, double > > polygon;
    for( unsigned int i = 0; i < points.size(); i++ )
    {
        boost::geometry::append( polygon, boost::make_tuple( points[i].x, points[i].y ) );
    }
    boost::geometry::append( polygon, boost::make_tuple( points[0].x, points[0].y ) ); // close polygon

    area = boost::geometry::area( polygon );

    boost::geometry::model::polygon< boost::tuple<double, double> > hull;
    boost::geometry::convex_hull( polygon, hull );

    convexArea = boost::geometry::area( hull );
}

/// constructor
/// @param image input image, is considered as a binary image ( all non-zero pixels are 1 )
region_properties::region_properties ( const cv::Mat& image, double minArea ):
    m_minArea( minArea )
{
    cv::Mat binary;
    if( image.channels() == 3 )
    {
        cv::cvtColor( image, binary, CV_RGB2GRAY );
    }
    else if( image.channels() == 1 )
    {
        binary = image;
    }
    else
    {
        COMMA_THROW( comma::exception, "incorrect number of channels, should be 1 or 3, not " << image.channels() );
    }
//     cv::Mat closed;
//     cv::morphologyEx( binary, closed, cv::MORPH_CLOSE, cv::Mat::ones( 3, 3, CV_8U) );
    std::vector< std::vector<cv::Point> > contours;
    cv::findContours( binary, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );
    
    for( unsigned int i = 0; i < contours.size(); i++ )
    {
        binary = cv::Scalar(0);
        cv::drawContours( binary, contours, i, cv::Scalar(0xFF), CV_FILLED );
        cv::Rect rect = cv::boundingRect( cv::Mat( contours[i]) );
        cv::Moments moments = cv::moments( binary( rect ), true );
        double x = moments.m10/moments.m00;
        double y = moments.m01/moments.m00;
        double area = moments.m00; // cv::countNonZero( binary( rect ) )
        if( area > m_minArea )
        {
            // see wikipedia, image moments
            double diff = moments.nu20 - moments.nu02;
            double a = 0.5 * ( moments.nu20 + moments.nu02 );
            double b = 0.5 * std::sqrt( 4 * moments.nu11 * moments.nu11 + diff * diff );
            double minEigenValue = a - b;
            double maxEigenValue = a + b;
    //         std::cerr << " min " << minEigenValue << " max " << maxEigenValue << std::endl;
            double theta = 0.5 * std::atan2( 2 * moments.nu11, diff );
            double eccentricity = 1;
            if( std::fabs( maxEigenValue ) > 1e-15 )
            {
                eccentricity = std::sqrt( 1 - minEigenValue / maxEigenValue );
            }

            double polygonArea;
            double convexArea;
            compute_area( contours[i], polygonArea, convexArea );
    //         std::cerr << " area " << area << " polygon " << polygonArea << " convex " << convexArea << std::endl;

            blob blob;
            blob.majorAxis = 2 * std::sqrt( moments.m00 * maxEigenValue );
            blob.minorAxis = 2 * std::sqrt( moments.m00 * minEigenValue );
            blob.orientation = theta;
            blob.centroid = cv::Point( x + rect.x, y + rect.y );
            blob.area = area;
            blob.eccentricity = eccentricity;
            blob.solidity = 0;
            if( std::fabs( convexArea ) > 1e-15 )
            {
                blob.solidity = polygonArea / convexArea;
            }
            m_blobs.push_back( blob );
        }
    }    
}

/// draw debug information on the image
void region_properties::show( cv::Mat& image, bool text )
{
    for( unsigned int i = 0; i < m_blobs.size(); i++ )
    {
        cv::Point centroid = m_blobs[i].centroid;
        cv::circle( image, centroid, 3, cv::Scalar( 0, 0, 255 ), 2 );
        std::stringstream s;
        s << i;
        if( text )
        {
            cv::putText( image, s.str(), centroid + cv::Point( 2, 2 ), cv::FONT_HERSHEY_PLAIN ,1, cv::Scalar( 0, 0, 255 ) );
        }
        cv::ellipse( image, centroid, cv::Size( m_blobs[i].majorAxis, m_blobs[i].minorAxis ), m_blobs[i].orientation * 180.0 / M_PI, 0, 360, cv::Scalar( 0, 255, 0 ) );
//         std::cerr << i << ": area " << m_blobs[i].area << " eccentricity " << m_blobs[i].eccentricity << " solidity " << m_blobs[i].solidity << std::endl;
    }
}

} } 


