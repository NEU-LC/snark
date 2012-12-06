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
//     leftCamera << 1632,   0,      631,
//                   0,      1630.9, 474.2,
//                   0,      0,      1;
    leftCamera << 5.3471311032432391e+02, 0., 3.3513838135674735e+02, 0.,
       5.3471311032432391e+02, 2.4020578137651341e+02, 0., 0., 1;
                  
    Eigen::Matrix< double, 5, 1 > leftDistortion;
//     leftDistortion << -0.43938, 0.21826, -0.00001, 0.00076, 0;
    leftDistortion << -2.7456815913629645e-01, -1.8329019064962277e-02, 0., 0., 0.;
    Eigen::Matrix3d rightCamera;
//     rightCamera << 1635.7, 0,      651.9,
//                   0,      1633.9, 463.4,
//                   0,      0,      1;

    rightCamera << 5.3471311032432391e+02, 0., 3.3401518911545526e+02, 0.,
       5.3471311032432391e+02, 2.4159041667844363e+02, 0., 0., 1.;
    Eigen::Matrix< double, 5, 1 > rightDistortion;
    rightDistortion << -2.8073450162365271e-01, 9.3000165783151290e-02, 0., 0., 0.;
//     rightDistortion <<  -0.44416, 0.23526, 0.00127, -0.00017, 0;


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
    rotation << 9.9975845371004723e-01, 5.2938494283307751e-03,
       -2.1330949194199030e-02, -4.9128856780201336e-03,
       9.9982820089904900e-01, 1.7872667436219597e-02,
       2.1421899766595000e-02, -1.7763553844914078e-02,
       9.9961270418356973e-01 ;
       
    Eigen::Vector3d translation( 0.24005, 0, 0 );
    translation <<  -3.3385325916025859e+00, 4.8752483611573305e-02,
       -1.0621381929002180e-01;

    snark::imaging::rectify_map rectify( leftCamera, leftDistortion, rightCamera, rightDistortion, left.cols, left.rows, rotation, translation );

    cv::Mat leftRectified = rectify.remap_left( left );
    cv::Mat rightRectified = rectify.remap_right( right );

    cv::imshow( "left", leftRectified );
    cv::imshow( "right", rightRectified );
    cv::imwrite( "left-rectified.png", leftRectified );
    cv::imwrite( "right-rectified.png", rightRectified );
    
    snark::imaging::point_cloud cloud( left.channels() );
    cv::Mat points = cloud.get( rectify.Q(), leftRectified, rightRectified );

    for( int i = 0; i < points.rows; i++ )
    {
       for( int j = 0; j < points.cols; j++ )
       {
            cv::Point3f point = points.at< cv::Point3f >( i, j );
            if( point.z < 100 )
            {
                cv::Vec3b color = leftRectified.at< cv::Vec3b >( i, j );
                std::cout << point.x << "," << point.y << "," << point.z << "," << (unsigned int)color[0] << "," << (unsigned int)color[1] << "," << (unsigned int)color[2] << std::endl;
            }
       }
    }

    cv::Mat disparity = cloud.disparity();
    std::cerr << " Q " << std::endl << rectify.Q() << std::endl;

    cv::Mat disparity8;
    unsigned int numberOfDisparities = 80;
    numberOfDisparities = ((left.cols/8) + 15) & -16;
    disparity.convertTo( disparity8, CV_8U, 255 / ( numberOfDisparities *16.0 ) );
    cv::imshow( "disparity", disparity8 );
    cv::imwrite( "disparity.png", disparity8 );
    cv::waitKey();
    
    return 0;
}
