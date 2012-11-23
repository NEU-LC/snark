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

#include <snark/imaging/stereo/point_cloud.h>
#include <snark/imaging/stereo/rectify_map.h>
#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <snark/math/rotation_matrix.h>



// convert bumblebee images to point cloud
int main( int ac, char** av )
{
    if( ac != 3 )
    {
        std::cerr << "usage: " << av[0] << " <left> <right> " << std::endl;
        return 1;
    }
    cv::Mat left = cv::imread( av[1], 0 );
    cv::Mat right = cv::imread( av[2], 0 );

    Eigen::Matrix3d leftCamera;
    leftCamera << 1632,   0,      631,
                  0,      1630.9, 474.2,
                  0,      0,      1;
                  
    Eigen::Matrix< double, 5, 1 > leftDistortion;
    leftDistortion << -0.43938, 0.21826, -0.00001, 0.00076, 0;
    Eigen::Matrix3d rightCamera;
    rightCamera << 1635.7, 0,      651.9,
                  0,      1633.9, 463.4,
                  0,      0,      1;
                  
    Eigen::Matrix< double, 5, 1 > rightDistortion;
    rightDistortion <<  -0.44416, 0.23526, 0.00127, -0.00017, 0;


//     Eigen::Vector3d leftPosition( 0.2229,-0.1283,-0.772 );
//     Eigen::Vector3d rightPosition( 0.2421,0.1018,-0.8288 );
//     Eigen::Vector3d leftAngle( 1.4621,0.0074,1.5435 );
//     Eigen::Vector3d rightAngle( 1.4262,0.0131,1.5456 );

//     Eigen::Vector3d angleDiff = rightAngle - leftAngle;
//     snark::rotation_matrix leftRotation( leftAngle );
//     snark::rotation_matrix rot( angleDiff );
//     Eigen::Matrix3d rotation = rot.rotation();
//     Eigen::Vector3d translation = leftRotation.rotation().transpose() * ( rightPosition - leftPosition );
    
    Eigen::Matrix3d rotation =  Eigen::Matrix3d::Identity();
    Eigen::Vector3d translation( 0.24005, 0, 0 );

    snark::imaging::rectify_map rectify( leftCamera, leftDistortion, rightCamera, rightDistortion, left.cols, left.rows, rotation, translation );

    cv::Mat leftRectified = rectify.remap_left( left );
    cv::Mat rightRectified = rectify.remap_right( right );

    cv::imshow( "left", leftRectified );
    cv::imshow( "right", rightRectified );
    
    snark::imaging::point_cloud cloud( rectify.Q(), left.channels() );
    cv::Mat points = cloud.get( leftRectified, rightRectified );

    for( unsigned int i = 0; i < points.rows; i++ )
    {
       for( unsigned int j = 0; j < points.cols; j++ )
       {
            cv::Point3f point = points.at< cv::Point3f >( i, j );
            if( point.z < 100 )
            {
                std::cout << point.x << "," << point.y << "," << point.z << std::endl;
            }
       }
    }

    cv::Mat disparity = cloud.disparity();

    cv::Mat disparity8;
    unsigned int numberOfDisparities = 80;
    numberOfDisparities = ((left.cols/8) + 15) & -16;
    disparity.convertTo( disparity8, CV_8U, 255 / ( numberOfDisparities *16.0 ) );
    cv::imshow( "disparity", disparity8 );
    cv::waitKey();
    
    return 0;
}
